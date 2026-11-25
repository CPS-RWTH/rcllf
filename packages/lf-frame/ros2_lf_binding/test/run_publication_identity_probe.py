#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression test: bounded cross-process publication identity and ownership."""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import uuid

PROCESS_TIMEOUT = 10
# Discovery under heavy parallel load can be slow. Both match waits
# (writer side in the probe, reader side here) use this bound, not PROCESS_TIMEOUT.
MATCH_TIMEOUT = float(os.environ.get('RCLLF_PROBE_MATCH_TIMEOUT_S', '30'))
# Publishes per reliability in the local-sync premise check.
LOCAL_SYNC_COUNT = 2000
EXPECTED_PACKAGES = {
    'ros-jazzy-fastrtps': '2.14.7-',
    'ros-jazzy-rclcpp': '28.1.22-',
    'ros-jazzy-rmw': '7.3.4-',
    'ros-jazzy-rmw-fastrtps-cpp': '8.4.4-',
}


def parse_events(output: str) -> list[dict]:
    events = []
    for line in output.splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        events.append(json.loads(line))
    return events


class ProbeFailure(RuntimeError):
    pass


class ProbeProcess:
    """
    A probe child whose stdout events are read while it runs.

    Publishers and the subscriber are released on observed discovery
    state (stdin commands), not after a fixed sleep.
    """

    def __init__(self, name: str, command: list[str], environment: dict[str, str]):
        self.name = name
        self.process = subprocess.Popen(
            command,
            env=environment,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.lines: list[str] = []
        self.events: list[dict] = []
        self.stderr_lines: list[str] = []
        self.reader_gate_ms: int | None = None
        self.changed = threading.Condition()
        self.readers = [
            threading.Thread(target=self._read_stdout, daemon=True),
            threading.Thread(target=self._read_stderr, daemon=True),
        ]
        for reader in self.readers:
            reader.start()

    def _read_stdout(self) -> None:
        for line in self.process.stdout:
            with self.changed:
                self.lines.append(line)
                if line.strip().startswith('{'):
                    self.events.append(json.loads(line))
                self.changed.notify_all()
        with self.changed:
            self.changed.notify_all()

    def _read_stderr(self) -> None:
        for line in self.process.stderr:
            self.stderr_lines.append(line)

    def output(self) -> str:
        return (
            f'stdout:\n{"".join(self.lines)}\n'
            f'stderr:\n{"".join(self.stderr_lines)}'
        )

    def wait_for(self, description: str, predicate, timeout: float = PROCESS_TIMEOUT) -> None:
        deadline = time.monotonic() + timeout
        with self.changed:
            while not predicate(self.events):
                remaining = deadline - time.monotonic()
                # stdout at EOF: the child can no longer produce the event.
                if remaining <= 0 or not self.readers[0].is_alive():
                    raise ProbeFailure(
                        f'{self.name}: {description} not observed '
                        f'(exit {self.process.poll()}):\n{self.output()}'
                    )
                self.changed.wait(min(remaining, 0.1))

    def wait_for_event(self, event: str, timeout: float = PROCESS_TIMEOUT) -> None:
        self.wait_for(
            event, lambda events: any(e.get('event') == event for e in events), timeout)

    def reader_match_mark(self) -> int:
        with self.changed:
            return len(self.events)

    def wait_for_new_reader_matches(self, mark: int, publishers: int) -> None:
        """
        Wait until the reader matched `publishers` new writers after event `mark`.

        Only a new writer raises the matched-publisher count on a unique topic. An
        exited writer can stay counted until its participant lease expires, so the
        gate counts increases instead of waiting for an absolute value.
        """
        def matched(events: list[dict]) -> bool:
            counts = [
                (index, event['publishers'])
                for index, event in enumerate(events)
                if event.get('event') == 'reader_matched'
            ]
            gained = sum(
                max(0, count - previous)
                for (_, previous), (index, count) in zip(counts, counts[1:])
                if index >= mark
            )
            return gained >= publishers
        self.wait_for(f'reader matched {publishers} new publisher(s)', matched, MATCH_TIMEOUT)

    def send(self, command: str) -> None:
        try:
            self.process.stdin.write(command + '\n')
            self.process.stdin.flush()
        except BrokenPipeError:
            pass  # The child already exited, and finish() reports its exit status.

    def match_report(self) -> str:
        writer = next(
            (event['match_ms'] for event in self.events if 'match_ms' in event), None)
        return (
            f'{self.name}: writer match {writer} ms, '
            f'reader gate waited {self.reader_gate_ms} ms'
        )

    def finish(self) -> list[dict]:
        self.process.stdin.close()
        self.process.wait(timeout=PROCESS_TIMEOUT)
        for reader in self.readers:
            reader.join(timeout=PROCESS_TIMEOUT)
        if self.process.returncode != 0:
            raise ProbeFailure(
                f'{self.name} failed ({self.process.returncode}):\n{self.output()}'
            )
        return self.events

    def kill(self) -> None:
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait()


class Runner:
    def __init__(self, executable: str):
        self.executable = executable
        self.environment = os.environ.copy()
        self.environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        self.environment['RMW_FASTRTPS_PUBLICATION_MODE'] = 'SYNCHRONOUS'
        self.results: dict[str, object] = {}

    def command(self, *arguments: object) -> list[str]:
        return [self.executable, *(str(argument) for argument in arguments)]

    def run(self, *arguments: object) -> tuple[list[dict], str]:
        completed = subprocess.run(
            self.command(*arguments),
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=PROCESS_TIMEOUT,
            check=False,
        )
        if completed.returncode != 0:
            command_text = ' '.join(self.command(*arguments))
            raise ProbeFailure(
                f'{command_text} failed ({completed.returncode}):\n'
                f'stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}'
            )
        return parse_events(completed.stdout), completed.stderr

    def topic(self, label: str) -> str:
        return f'/issue85/{label}_{uuid.uuid4().hex}'

    def start_subscriber(
        self, label: str, topic: str, expected: int, depth: int, hold: bool, capacity: int
    ) -> ProbeProcess:
        subscriber = ProbeProcess(
            f'subscriber {label}',
            self.command('subscriber', topic, expected, depth, int(hold), capacity),
            self.environment,
        )
        subscriber.wait_for_event('subscriber_ready')
        return subscriber

    def start_gated_publisher(
        self, topic: str, epoch: str, count: int, processes: list[ProbeProcess],
    ) -> ProbeProcess:
        publisher = ProbeProcess(
            f'publisher {epoch}',
            self.command('publisher', topic, epoch, count, 1, 0, 1),
            self.environment,
        )
        # Registered before the wait so a failure report includes its match time.
        processes.append(publisher)
        publisher.wait_for_event('writer_matched', MATCH_TIMEOUT + PROCESS_TIMEOUT)
        return publisher

    @staticmethod
    def gate_on_reader(
        subscriber: ProbeProcess, mark: int, publishers: list[ProbeProcess],
    ) -> None:
        start = time.monotonic()
        try:
            subscriber.wait_for_new_reader_matches(mark, len(publishers))
        finally:
            for publisher in publishers:
                publisher.reader_gate_ms = int((time.monotonic() - start) * 1000)

    @staticmethod
    def publish_events(publisher: ProbeProcess) -> list[dict]:
        events = publisher.finish()
        if not any(event.get('event') == 'all_acked' for event in events):
            raise ProbeFailure(f'{publisher.name} exited without all_acked')
        return [event for event in events if event.get('event') == 'publish']

    @staticmethod
    def stop(processes: list[ProbeProcess], error: BaseException) -> None:
        for process in processes:
            process.kill()
        if isinstance(error, ProbeFailure) and processes:
            # The subscriber's view (its reader_matched trace) explains most failures.
            subscriber = processes[0]
            for reader in subscriber.readers:
                reader.join(timeout=PROCESS_TIMEOUT)
            matches = '\n'.join(process.match_report() for process in processes[1:])
            error.args = (
                f'{error.args[0]}\nmatch times:\n{matches}\n'
                f'{subscriber.name} {subscriber.output()}',
            )

    def run_exchange(
        self,
        label: str,
        publishers: list[tuple[str, int]],
        *,
        expected: int,
        depth: int,
        hold: bool,
        capacity: int,
    ) -> tuple[list[dict], list[dict]]:
        topic = self.topic(label)
        processes: list[ProbeProcess] = []
        try:
            subscriber = self.start_subscriber(label, topic, expected, depth, hold, capacity)
            processes.append(subscriber)
            publish_events: list[dict] = []
            for epoch, count in publishers:
                mark = subscriber.reader_match_mark()
                publisher = self.start_gated_publisher(topic, epoch, count, processes)
                self.gate_on_reader(subscriber, mark, [publisher])
                publisher.send('publish')
                publish_events.extend(self.publish_events(publisher))
            subscriber.send('take' if hold else 'matched')
            events = subscriber.finish()
        except BaseException as error:
            self.stop(processes, error)
            raise
        return publish_events, events

    def run_parallel_exchange(self) -> tuple[list[dict], list[dict]]:
        topic = self.topic('two_live_publishers')
        processes: list[ProbeProcess] = []
        try:
            subscriber = self.start_subscriber('parallel', topic, 2, 10, False, 2)
            processes.append(subscriber)
            mark = subscriber.reader_match_mark()
            publishers = [
                self.start_gated_publisher(topic, epoch, 1, processes)
                for epoch in ('publisher-a', 'publisher-b')
            ]
            self.gate_on_reader(subscriber, mark, publishers)
            subscriber.send('matched')
            for publisher in publishers:
                publisher.send('publish')
            publish_events: list[dict] = []
            for publisher in publishers:
                publish_events.extend(self.publish_events(publisher))
            events = subscriber.finish()
        except BaseException as error:
            self.stop(processes, error)
            raise
        return publish_events, events

    @staticmethod
    def assert_ownership(events: list[dict], expected: int, capacity: int) -> None:
        takes = [event for event in events if event.get('event') == 'take']
        callbacks = [event for event in events if event.get('event') == 'callback']
        summaries = [event for event in events if event.get('event') == 'subscriber_summary']
        if len(takes) != expected or len(callbacks) != expected or len(summaries) != 1:
            raise ProbeFailure('missing take/callback/summary ownership evidence')
        summary = summaries[0]
        if not summary['pointer_continuity']:
            raise ProbeFailure('retained pointer changed between take and callback')
        if summary['create_count'] != expected or summary['take_attempts'] != expected:
            raise ProbeFailure('admitted sample did not use exactly one create and one take')
        if summary['dispatch_take_count'] != 0:
            raise ProbeFailure('dispatch performed a second destructive take')
        if summary['max_retained'] > capacity:
            raise ProbeFailure('retention exceeded configured capacity')
        for taken, callback in zip(takes, callbacks):
            if taken['address'] != callback['address']:
                raise ProbeFailure('take/callback address mismatch')
            if taken['publication_sequence_number'] != callback['publication_sequence_number']:
                raise ProbeFailure('take/callback metadata mismatch')

    @staticmethod
    def assert_exact_mapping(publishes: list[dict], receives: list[dict]) -> None:
        published = {
            (event['publisher_gid'], event['adapter_ordinal'])
            for event in publishes
        }
        received = {
            (event['publisher_gid'], event['publication_sequence_number'])
            for event in receives
            if event.get('event') == 'take'
        }
        if published != received:
            raise ProbeFailure(
                'negative feasibility: adapter ordinal does not exactly equal the '
                f'received publication sequence number; publish={sorted(published)}, '
                f'receive={sorted(received)}'
            )

    def zero_fanout(self) -> None:
        events, _ = self.run('publisher', self.topic('zero'), 'zero', 1, 0, 0)
        publishes = [event for event in events if event.get('event') == 'publish']
        if len(publishes) != 1 or publishes[0]['matched_subscriptions'] != 0:
            raise ProbeFailure('zero-fan-out publication did not report zero local matches')
        self.results['zero_local_fanout'] = 'passed'

    def one_fanout_and_mapping(self) -> None:
        publishes, events = self.run_exchange(
            'one', [('one', 3)], expected=3, depth=10, hold=False, capacity=3
        )
        if any(event['matched_subscriptions'] != 1 for event in publishes):
            observed = ', '.join(
                f"ordinal={event.get('adapter_ordinal')} "
                f"matched_subscriptions={event.get('matched_subscriptions')} "
                f"stable={event.get('matched_subscriptions_stable')} "
                f"stabilization_us={event.get('match_stabilization_us')} "
                f"publish_to_sample_us={event.get('publish_to_sample_us')}"
                for event in publishes
            )
            raise ProbeFailure(
                'one-fan-out publication did not report exactly one local match; '
                f'observed per publish: [{observed}]'
            )
        self.assert_ownership(events, 3, 3)
        self.assert_exact_mapping(publishes, events)
        payloads = {
            (event['payload_frame_id'], event['stamp_sec'], event['stamp_nanosec'])
            for event in publishes
        }
        if len(payloads) != 1:
            raise ProbeFailure('publisher changed payload or application stamp across publishes')
        self.results['one_local_fanout'] = 'passed'
        self.results['ordinal_equals_publication_sequence'] = 'passed'
        self.results['retained_pointer_continuity'] = 'passed'
        self.results['retention_bound'] = 3

    def two_live_publishers(self) -> None:
        publishes, events = self.run_parallel_exchange()
        self.assert_ownership(events, 2, 2)
        self.assert_exact_mapping(publishes, events)
        gids = {event['publisher_gid'] for event in publishes}
        if len(publishes) != 2 or len(gids) != 2:
            raise ProbeFailure('two live publishers did not have distinct GIDs')
        self.results['two_live_publishers_identical_payload'] = 'passed'

    def publisher_restart(self) -> None:
        publishes, events = self.run_exchange(
            'restart',
            [('epoch-a', 1), ('epoch-b', 1)],
            expected=2,
            depth=10,
            hold=False,
            capacity=2,
        )
        self.assert_ownership(events, 2, 2)
        self.assert_exact_mapping(publishes, events)
        gids = {event['publisher_gid'] for event in publishes}
        epochs = {event['epoch'] for event in publishes}
        if len(gids) != 2 or epochs != {'epoch-a', 'epoch-b'}:
            raise ProbeFailure('publisher restart did not produce a new epoch and GID')
        self.results['publisher_restart_new_epoch_gid'] = 'passed'

    def environment_pins(self) -> dict[str, str]:
        events, _ = self.run('environment')
        runtime = next(
            (event for event in events if event.get('event') == 'environment'),
            None,
        )
        if runtime is None or runtime['rmw_implementation'] != 'rmw_fastrtps_cpp':
            raise ProbeFailure(f'unexpected runtime RMW implementation: {runtime}')
        completed = subprocess.run(
            ['dpkg-query', '-W', *EXPECTED_PACKAGES],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=PROCESS_TIMEOUT,
            check=False,
        )
        if completed.returncode != 0:
            raise ProbeFailure(f'could not identify pinned binary packages: {completed.stderr}')
        installed = {}
        for line in completed.stdout.splitlines():
            package, version = line.split('\t', maxsplit=1)
            installed[package] = version
        mismatches = {
            package: installed.get(package)
            for package, prefix in EXPECTED_PACKAGES.items()
            if not installed.get(package, '').startswith(prefix)
        }
        if mismatches:
            raise ProbeFailure(f'refusing unpinned binary versions: {mismatches}')
        return installed

    def stale_queue_falsification(self) -> None:
        publishes, events = self.run_exchange(
            'stale', [('stale', 2)], expected=1, depth=10, hold=True, capacity=1
        )
        self.assert_ownership(events, 1, 1)
        taken = next(event for event in events if event.get('event') == 'take')
        latest = max(event['adapter_ordinal'] for event in publishes)
        if taken['publication_sequence_number'] == latest:
            raise ProbeFailure('falsification failed: unqualified take selected latest sample')
        if taken['publication_sequence_number'] != 1 or latest != 2:
            raise ProbeFailure('unexpected stale-queue sequence evidence')
        self.results['legacy_unqualified_take_stale'] = {
            'selected': taken['publication_sequence_number'],
            'expected_latest': latest,
        }

    def keep_last_loss(self) -> None:
        publishes, events = self.run_exchange(
            'loss', [('loss', 5)], expected=2, depth=2, hold=True, capacity=2
        )
        self.assert_ownership(events, 2, 2)
        sequences = [
            event['publication_sequence_number']
            for event in events
            if event.get('event') == 'take'
        ]
        if sequences != [4, 5]:
            raise ProbeFailure(f'KeepLast(2) did not expose the expected [4, 5] gap: {sequences}')
        if len(publishes) != 5:
            raise ProbeFailure('loss scenario did not execute all five publish operations')
        self.results['keep_last_loss_gap'] = sequences

    def classifications(self) -> None:
        refusal_events, _ = self.run('refusals')
        refusal = next(
            (event for event in refusal_events if event.get('event') == 'refusal_summary'),
            None,
        )
        if refusal is None or not refusal['passed']:
            raise ProbeFailure(f'refusal checks were not all detected: {refusal}')
        for origin in ('helper_thread', 'reentrant', 'lifecycle', 'loaned', 'uninstrumented'):
            if refusal[origin] != 'refused':
                raise ProbeFailure(f'{origin} was not fail-closed')
        intra_events, _ = self.run('intra-process')
        intra = next(
            (event for event in intra_events if event.get('event') == 'intra_process_summary'),
            None,
        )
        if intra is None or intra['classification'] != 'refused':
            raise ProbeFailure(
                f'intra-process path was not proven and must remain refused: {intra}'
            )
        self.results['refused_origins'] = [
            'helper_thread', 'reentrant', 'lifecycle', 'loaned', 'uninstrumented'
        ]
        self.results['intra_process'] = {
            'classification': intra['classification'],
            'reason': intra['reason'],
        }

    def loaned_sequence(self) -> None:
        # Interleaved loaned and ordinary writes share one sequence.
        events, _ = self.run('loaned-sequence', self.topic('loaned'), 200)
        summary = next(
            (event for event in events if event.get('event') == 'loaned_sequence_summary'),
            None,
        )
        if summary is None or not summary['passed']:
            raise ProbeFailure(f'loaned writes did not keep ordinal == sequence: {summary}')
        self.results['loaned_ordinal_equals_publication_sequence'] = 'passed'

    def local_sync_events(self, reliability: str) -> dict:
        # Exit code 8 is the probe's own failure verdict, so the
        # summary is read whatever the exit code.
        # The child waits up to MATCH_TIMEOUT for both
        # sides to match before it publishes, so its bound is that plus the run.
        completed = subprocess.run(
            self.command('local-sync', self.topic('local_sync'), LOCAL_SYNC_COUNT, reliability),
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=MATCH_TIMEOUT + PROCESS_TIMEOUT,
            check=False,
        )
        summary = next(
            (
                event for event in parse_events(completed.stdout)
                if event.get('event') == 'local_sync_summary'
            ),
            None,
        )
        if summary is None:
            raise ProbeFailure(
                f'local-sync produced no summary ({completed.returncode}):\n'
                f'stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}'
            )
        return summary

    def local_sync(self) -> None:
        # The publish-time take premise. Every publish must be in
        # the other node's reader when rcl_publish returns.
        for reliability in ('reliable', 'best_effort'):
            summary = self.local_sync_events(reliability)
            if not summary['passed']:
                raise ProbeFailure(f"{summary['reason']}: {summary}")
            self.results[f'local_sync_{reliability}'] = {
                'publishes': summary['publishes'], 'found': summary['found'],
            }

    def execute(self) -> dict[str, object]:
        packages = self.environment_pins()
        self.zero_fanout()
        self.one_fanout_and_mapping()
        self.two_live_publishers()
        self.publisher_restart()
        self.stale_queue_falsification()
        self.keep_last_loss()
        self.classifications()
        self.loaned_sequence()
        self.local_sync()
        return {
            'issue': 85,
            'rmw': 'rmw_fastrtps_cpp',
            'publication_mode': 'SYNCHRONOUS',
            'packages': packages,
            'status': 'passed',
            'results': self.results,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe', required=True)
    parser.add_argument(
        '--local-sync-only',
        action='store_true',
        help='run only the local-sync premise check (with the env publication mode)',
    )
    arguments = parser.parse_args()
    try:
        runner = Runner(arguments.probe)
        if arguments.local_sync_only:
            # Falsification: RMW_FASTRTPS_PUBLICATION_MODE from the caller wins.
            mode = os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', 'SYNCHRONOUS')
            runner.environment['RMW_FASTRTPS_PUBLICATION_MODE'] = mode
            runner.local_sync()
            summary = {'issue': 698, 'publication_mode': mode, 'status': 'passed',
                       'results': runner.results}
        else:
            summary = runner.execute()
    except (ProbeFailure, subprocess.TimeoutExpired, json.JSONDecodeError) as error:
        print(json.dumps({'issue': 85, 'status': 'failed', 'reason': str(error)}))
        return 1
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
