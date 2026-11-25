# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Source scan for message_filters synchronizers (route A).

``message_filters::Subscriber::subscribe(node, "topic")`` creates its rclcpp
subscription inside the library, so the libclang walk in ``clang_parser.py``
(which keys on ``create_subscription`` calls in user code) never sees it. The
AST also degrades when the message_filters headers are missing, so this scan
works on the comment-stripped source text of the node's .cpp and header, the
same robustness argument as ``_scan_detected_includes``.

Admitted form (everything else is reported in ``unsupported``):

* ``message_filters::Subscriber<M>`` plain members, each subscribed exactly
  once with ``member.subscribe(this, "topic"[, qos[, options]])`` or the
  ``member(this, "topic"[, qos[, options]])`` constructor form,
* a ``sync_policies::ExactTime`` or ``ApproximateTime`` policy with two
  message types, or rcllf's ``rcllf_message_filters::sync_policies::
  BoundedNearestToAnchor``, named by a ``using``/``typedef`` alias
  or spelled inline,
* a ``message_filters::Synchronizer<P>`` member built from ``P(N)`` (for the
  BNA policy ``P(delta, lateness0, lateness1, capacity0, capacity1,
  retention, timeout)`` with integer literal arguments) and exactly two of
  those Subscriber members (``std::make_shared``,
  ``std::make_unique`` or a plain member's init-list),
* one ``registerCallback(std::bind(&Class::cb, this, _1, _2))`` (or
  ``registerCallback(&Class::cb, this)``) on that synchronizer,
* optionally one observer ``registerCallback(std::bind(&Class::m, this, _1))``
  per Subscriber (an arrival log). The library calls it from the
  same rclcpp subscription callback that feeds the synchronizer, so it runs
  inside that input's reaction; its effects are merged into the input's
  callback and it may only read and log (``observer_refusal``).

Each input then becomes one subscription whose callback is the synthetic name
``<cb>_via_<member>``. At run time the binding dispatches the rclcpp
subscription the library created, by topic, inside that input's LF reaction,
and the synchronizer calls ``cb`` synchronously within it.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from colcon_lf.node_analysis.models import (
    MessageFiltersInfo,
    MessageFiltersSyncInfo,
)

BNA_POLICY = "BoundedNearestToAnchor"
ADMITTED_POLICIES = ("ExactTime", "ApproximateTime", BNA_POLICY)
BNA_PARAM_NAMES = (
    "delta", "lateness0", "lateness1", "capacity0", "capacity1", "retention", "timeout",
)
# message_filters names the admitted form uses. Any other
# ``message_filters::X`` (Cache, TimeSequencer, Chain, PassThrough, ...) is
# reported as unsupported.
_ADMITTED_NAMES = frozenset({"Subscriber", "Synchronizer", "sync_policies"})


@dataclass
class ScannedInput:
    """One admitted synchronizer input (becomes a SubscriberInfo)."""

    member: str
    topic: str
    message_type: str
    qos_args: str | None          # raw third subscribe argument, None if absent
    sync_callback: str
    callback: str                 # synthetic per-input callback name
    observer: str | None = None   # one-argument Subscriber callback


@dataclass
class ScanResult:
    info: MessageFiltersInfo | None
    inputs: list[ScannedInput] = field(default_factory=list)


def queue_parameter(policy_arg: str, code: str) -> str | None:
    """The ROS parameter a non-literal policy queue size reads, or None.

    ``Policy(static_cast<T>(x))`` or
    ``Policy(x)`` where *x* is assigned exactly once in the node source, from
    ``declare_parameter<T>("name", ...)``. The generator then takes the
    parameter's resolved value (launch, params file or declared default) as
    the queue size Q of the ApproximateTime burst bound. Any other form gives
    None and the edge keeps no proven bound.
    """
    m = re.fullmatch(r"[\w:<>\s]*\(\s*(.*)\s*\)", policy_arg.strip(), re.S)
    if m is None:
        return None
    inner = m.group(1).strip()
    cast = re.fullmatch(r"static_cast\s*<[^<>]*>\s*\(\s*(\w+)\s*\)", inner)
    ident = cast.group(1) if cast else inner
    if not re.fullmatch(r"[A-Za-z_]\w*", ident):
        return None
    writes = re.findall(rf"(?<![\w.>]){re.escape(ident)}\s*=(?!=)\s*([^;]*);", code)
    if len(writes) != 1:
        return None
    decl = re.fullmatch(
        r"(?:this\s*->\s*)?declare_parameter\s*(?:<[^<>]*>)?\s*\(\s*\"([^\"]+)\".*\)",
        writes[0].strip(), re.S)
    return decl.group(1) if decl else None


def strip_comments(text: str) -> str:
    """Remove // and /* */ comments while keeping string literals intact."""
    out: list[str] = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '"' or c == "'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            out.append(" ")
            i = n if j < 0 else j + 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def _balanced(text: str, start: int, open_ch: str, close_ch: str) -> tuple[str, int] | None:
    """Return (inner text, index after close) for the bracket opening at *start*."""
    if start >= len(text) or text[start] != open_ch:
        return None
    depth = 0
    i = start
    while i < len(text):
        c = text[i]
        if c == '"':
            j = i + 1
            while j < len(text) and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            i = j + 1
            continue
        if c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return text[start + 1:i], i + 1
        i += 1
    return None


def _split_args(inner: str) -> list[str]:
    """Split a call or template argument list on top-level commas."""
    args: list[str] = []
    depth = 0
    cur: list[str] = []
    i = 0
    while i < len(inner):
        c = inner[i]
        if c == '"':
            j = i + 1
            while j < len(inner) and inner[j] != '"':
                j += 2 if inner[j] == "\\" else 1
            cur.append(inner[i:j + 1])
            i = j + 1
            continue
        if inner.startswith("->", i):
            cur.append("->")
            i += 2
            continue
        if c in "(<[{":
            depth += 1
        elif c in ")>]}":
            depth -= 1
        if c == "," and depth == 0:
            args.append("".join(cur).strip())
            cur = []
        else:
            cur.append(c)
        i += 1
    tail = "".join(cur).strip()
    if tail or args:
        args.append(tail)
    return args


def _norm(spelling: str) -> str:
    return re.sub(r"\s+", "", spelling)


_STRING_LIT = re.compile(r'^"((?:[^"\\]|\\.)*)"$')
_PLACEHOLDER = re.compile(r"^(?:std::placeholders::)?_(\d)$")


def _parse_callback(args: list[str], class_name: str) -> str | None:
    """Return the method name for an admitted registerCallback argument list."""
    def method_of(ref: str) -> str | None:
        m = re.fullmatch(r"&\s*(?:(?:\w+::)*(\w+)::)?(\w+)", ref.strip())
        if not m or (m.group(1) and m.group(1) != class_name):
            return None
        return m.group(2)

    if len(args) == 2 and _norm(args[1]) == "this":
        return method_of(args[0])
    if len(args) != 1:
        return None
    m = re.fullmatch(r"std::bind\s*(\(.*\))", args[0].strip(), re.S)
    if not m:
        return None
    inner = _balanced(m.group(1), 0, "(", ")")
    if inner is None:
        return None
    bargs = _split_args(inner[0])
    if len(bargs) != 4 or _norm(bargs[1]) != "this":
        return None
    p1, p2 = _PLACEHOLDER.match(_norm(bargs[2])), _PLACEHOLDER.match(_norm(bargs[3]))
    if not (p1 and p2 and p1.group(1) == "1" and p2.group(1) == "2"):
        return None
    return method_of(bargs[0])


def _parse_observer(args: list[str], class_name: str) -> str | None:
    """Method name of ``std::bind(&Class::m, this, _1)``, else None."""
    if len(args) != 1:
        return None
    m = re.fullmatch(r"std::bind\s*(\(.*\))", args[0].strip(), re.S)
    inner = _balanced(m.group(1), 0, "(", ")") if m else None
    if inner is None:
        return None
    bargs = _split_args(inner[0])
    p1 = _PLACEHOLDER.match(_norm(bargs[2])) if len(bargs) == 3 else None
    if not (p1 and p1.group(1) == "1" and _norm(bargs[1]) == "this"):
        return None
    ref = re.fullmatch(r"&\s*(?:(?:\w+::)*(\w+)::)?(\w+)", bargs[0].strip())
    if not ref or (ref.group(1) and ref.group(1) != class_name):
        return None
    return ref.group(2)


_OBSERVER_EFFECTS = ("publishers_used", "service_clients_used", "action_clients_used",
                     "spawns_callbacks", "feedback_published", "result_published")


def observer_refusal(cb) -> str | None:
    """Why an observer callback is refused, or None when it only reads and logs.

    An observer runs inside its input's reaction like the synchronizer
    callback, but its effects are not declared anywhere else: a publish or a
    client call there would be an undeclared output of the reaction.
    """
    for name in _OBSERVER_EFFECTS:
        got = getattr(cb, name, None) or []
        if got:
            return (f"observer {cb.name} has effects ({name}: {', '.join(got)}); an "
                    "observer on a synchronizer input may only read and log")
    return None


def _template_after(text: str, pos: int) -> tuple[str, int] | None:
    """Parse ``<...>`` starting at the first non-space char at or after *pos*."""
    while pos < len(text) and text[pos].isspace():
        pos += 1
    return _balanced(text, pos, "<", ">")


def _call_args_after(text: str, pos: int) -> tuple[list[str], int] | None:
    while pos < len(text) and text[pos].isspace():
        pos += 1
    if pos < len(text) and text[pos] in "({":
        close = ")" if text[pos] == "(" else "}"
        got = _balanced(text, pos, text[pos], close)
        if got is not None:
            return _split_args(got[0]), got[1]
    return None


def scan(sources: list[str], class_name: str) -> ScanResult:
    """Scan the node's source texts (.cpp and header) for message_filters use."""
    text = "\n".join(strip_comments(s) for s in sources)
    if "message_filters" not in text:
        return ScanResult(info=None)

    unsupported: list[str] = []

    def refuse(reason: str) -> None:
        if reason not in unsupported:
            unsupported.append(reason)

    # Includes name headers, not C++ entities; ignore them for the name check.
    code = re.sub(r"^\s*#\s*include[^\n]*", "", text, flags=re.M)
    bare = re.sub(r'"(?:[^"\\]|\\.)*"', '""', code)
    if re.search(r"\bmessage_filters\b(?!\s*::)", bare):
        refuse("message_filters is referenced other than as message_filters::Name "
               "(namespace alias or using-directive), which the analyzer cannot follow")
    for name in sorted(set(re.findall(r"message_filters\s*::\s*(\w+)", code))):
        if name not in _ADMITTED_NAMES:
            refuse(f"message_filters::{name} is not supported "
                   "(only Subscriber inputs feeding a two-input Synchronizer are)")
    for name in sorted(set(re.findall(r"sync_policies\s*::\s*(\w+)", code))):
        if name not in ADMITTED_POLICIES:
            refuse(f"sync policy {name} is not supported "
                   "(only ExactTime, ApproximateTime and rcllf's BoundedNearestToAnchor are)")

    # ---- policy aliases: alias -> (policy, n message types) --------------
    policies: dict[str, tuple[str, int]] = {}

    def policy_spelling(spelling: str) -> tuple[str, int] | None:
        m = re.match(
            r"\s*(?:(?:rcllf_)?message_filters\s*::\s*)?sync_policies\s*::\s*(\w+)\s*", spelling)
        if not m:
            return None
        tmpl = _template_after(spelling, m.end())
        n_args = len(_split_args(tmpl[0])) if tmpl else 0
        return m.group(1), n_args

    for m in re.finditer(r"\busing\s+(\w+)\s*=\s*", code):
        got = policy_spelling(code[m.end():])
        if got:
            policies[m.group(1)] = got
    for m in re.finditer(r"\btypedef\s+", code):
        got = policy_spelling(code[m.end():])
        if got:
            tail = re.match(
                r"\s*(?:(?:rcllf_)?message_filters\s*::\s*)?sync_policies\s*::\s*\w+\s*",
                code[m.end():])
            tmpl = _template_after(code, m.end() + tail.end())
            if tmpl:
                alias = re.match(r"\s*(\w+)\s*;", code[tmpl[1]:])
                if alias:
                    policies[alias.group(1)] = got

    def resolve_policy(spelling: str) -> tuple[str, int] | None:
        key = _norm(spelling)
        if key in policies:
            return policies[key]
        return policy_spelling(spelling)

    # ---- Subscriber members: name -> message type ------------------------
    subscribers: dict[str, str] = {}
    for m in re.finditer(r"message_filters\s*::\s*Subscriber\b", code):
        tmpl = _template_after(code, m.end())
        decl = re.match(r"\s*(\w+)\s*;", code[tmpl[1]:]) if tmpl else None
        before = code[max(0, m.start() - 40):m.start()]
        if decl is None or re.search(r"<\s*$", before):
            refuse("message_filters::Subscriber is only supported as a plain "
                   "class member (not by pointer, local variable or template argument)")
            continue
        targs = _split_args(tmpl[0])
        subscribers[decl.group(1)] = _norm(targs[0]) if targs else ""

    # ---- Synchronizer members and their construction ---------------------
    syncs: dict[str, tuple[str, int] | None] = {}
    sync_ptr: dict[str, bool] = {}
    for m in re.finditer(r"message_filters\s*::\s*Synchronizer\b", code):
        tmpl = _template_after(code, m.end())
        if tmpl is None:
            continue
        tail = code[tmpl[1]:]
        plain = re.match(r"\s*(\w+)\s*;", tail)
        ptr = re.match(r"\s*>\s*(\w+)\s*;", tail)
        before = code[max(0, m.start() - 60):m.start()]
        if plain and not re.search(r"<\s*$", before):
            syncs[plain.group(1)] = resolve_policy(tmpl[0])
            sync_ptr[plain.group(1)] = False
        elif ptr and re.search(r"std\s*::\s*(?:shared|unique)_ptr\s*<\s*$", before):
            syncs[ptr.group(1)] = resolve_policy(tmpl[0])
            sync_ptr[ptr.group(1)] = True

    built: dict[str, tuple[int | None, list[str]]] = {}
    built_policy_args: dict[str, str] = {}  # member -> the policy argument as written
    queue_params: dict[str, str] = {}  # member -> ROS parameter its queue size reads

    def record_build(member: str, args: list[str]) -> None:
        if member in built:
            refuse(f"synchronizer {member} is constructed more than once")
            return
        if not args:
            refuse(f"synchronizer {member} is built without a policy argument")
            return
        q = re.search(r"\(\s*(\d+)\s*\)$", args[0].strip())
        built[member] = (int(q.group(1)) if q else None, [_norm(a) for a in args[1:]])
        built_policy_args[member] = args[0].strip()
        if q is None:
            param = queue_parameter(args[0], code)
            if param is not None:
                queue_params[member] = param

    for member in syncs:
        if sync_ptr[member]:
            for m in re.finditer(
                rf"(?<![\w.>]){re.escape(member)}\s*=\s*std\s*::\s*make_(?:shared|unique)\s*"
                r"<\s*message_filters\s*::\s*Synchronizer\b", code,
            ):
                tmpl = _template_after(code, m.end())
                close = re.match(r"\s*>", code[tmpl[1]:]) if tmpl else None
                got = _call_args_after(code, tmpl[1] + close.end()) if close else None
                if got:
                    record_build(member, got[0])
            if re.search(rf"\b{re.escape(member)}\s*(?:\.|->)\s*reset\s*\(", code) or re.search(
                    rf"(?<![\w.>]){re.escape(member)}\s*=(?!=)(?!\s*std\s*::\s*make_)", code):
                refuse(f"synchronizer {member} is created other than by "
                       "std::make_shared or std::make_unique")
        else:
            for m in re.finditer(rf"[:,]\s*{re.escape(member)}\s*(?=[({{])", code):
                got = _call_args_after(code, m.end())
                if got:
                    record_build(member, got[0])

    # ---- registerCallback ------------------------------------------------
    callbacks: dict[str, list[str | None]] = {}
    observers: dict[str, list[str | None]] = {}
    for m in re.finditer(r"(\w+)\s*(?:\.|->)\s*registerCallback\s*(?=\()", code):
        target = m.group(1)
        got = _call_args_after(code, m.end())
        if target in subscribers:
            observers.setdefault(target, []).append(
                _parse_observer(got[0], class_name) if got else None)
            continue
        cb = _parse_callback(got[0], class_name) if got else None
        callbacks.setdefault(target, []).append(cb)
    bad_observers = set()
    for target, obs in observers.items():
        if len(obs) != 1 or obs[0] is None:
            refuse(f"Subscriber {target} admits one observer registerCallback("
                   f"std::bind(&Class::method, this, _1)), found {len(obs)}")
            bad_observers.add(target)

    if re.search(r"\bconnectInput\s*\(", code):
        refuse("connectInput is not supported "
               "(pass the two Subscriber members to the Synchronizer constructor)")

    # ---- subscribe calls and constructor form ----------------------------
    sub_calls: dict[str, list[list[str]]] = {}
    for member in subscribers:
        for m in re.finditer(rf"(?<![\w.>]){re.escape(member)}\s*(?:\.|->)\s*subscribe\s*(?=\()",
                             code):
            got = _call_args_after(code, m.end())
            if got:
                sub_calls.setdefault(member, []).append(got[0])
        for m in re.finditer(rf"[:,]\s*{re.escape(member)}\s*(?=[({{])", code):
            got = _call_args_after(code, m.end())
            if got and got[0] and got[0] != [""]:
                sub_calls.setdefault(member, []).append(got[0])

    # One Subscriber member is one runtime subscription, so it can carry one
    # reaction ID. Feeding two inputs, or unsubscribing it, breaks that.
    feeds: dict[str, int] = {}
    for _queue, ins in built.values():
        for sub in ins:
            feeds[sub] = feeds.get(sub, 0) + 1
    shared = {sub for sub, n in feeds.items() if n > 1 and sub in subscribers}
    for sub in sorted(shared):
        refuse(f"Subscriber {sub} feeds {feeds[sub]} synchronizer inputs "
               "(one Subscriber member per input, it is one subscription)")
    unsubscribed = {
        sub for sub in subscribers
        if re.search(rf"(?<![\w.>]){re.escape(sub)}\s*(?:\.|->)\s*unsubscribe\s*\(", code)
    }
    for sub in sorted(unsubscribed):
        refuse(f"Subscriber {sub} calls unsubscribe() "
               "(an input must stay subscribed for the life of the node)")

    # ---- assemble ----------------------------------------------------------
    admitted: list[MessageFiltersSyncInfo] = []
    inputs: list[ScannedInput] = []
    used: set[str] = set()
    for member, pol in syncs.items():
        if pol is None:
            refuse(f"synchronizer {member} uses a policy the analyzer cannot resolve")
            continue
        policy, n_types = pol
        if policy not in ADMITTED_POLICIES:
            continue  # already refused by the policy name check
        bna = policy == BNA_POLICY
        if n_types != 2:
            refuse(f"{'input_count: ' if bna else ''}synchronizer {member} has {n_types} inputs "
                   "(only two-input synchronizers are supported)")
            continue
        if member not in built:
            refuse(f"synchronizer {member} is not constructed in a recognised form "
                   "(Policy(N), sub_a, sub_b)")
            continue
        queue, ins = built[member]
        used.update(ins)
        if len(ins) != 2:
            refuse(f"synchronizer {member} is constructed with {len(ins)} inputs "
                   "(only two-input synchronizers are supported)")
            continue
        bad = [i for i in ins if i not in subscribers]
        if bad:
            refuse(f"synchronizer {member} input {bad[0]} is not a "
                   "message_filters::Subscriber member (chained filters are not supported)")
            continue
        if any(i in shared or i in unsubscribed for i in ins):
            continue  # refused above, naming the Subscriber
        bna_params = None
        if bna:
            if any(a.policy == BNA_POLICY for a in admitted):
                refuse(f"bna_multiple: synchronizer {member} is a second "
                       "BoundedNearestToAnchor synchronizer in this node (one per node)")
                continue
            bna_params = bna_literal_params(built_policy_args.get(member, ""))
            if bna_params is None:
                refuse(f"bna_params_not_literal: synchronizer {member} must build its policy "
                       "as BoundedNearestToAnchor(delta, lateness0, lateness1, capacity0, "
                       "capacity1, retention, timeout) with valid integer literal "
                       "nanoseconds")
                continue
        cbs = callbacks.get(member, [])
        if len(cbs) != 1 or cbs[0] is None:
            refuse(f"synchronizer {member} needs exactly one registerCallback("
                   "std::bind(&Class::method, this, _1, _2))")
            continue
        sync_inputs: list[ScannedInput] = []
        for sub in ins:
            if sub in bad_observers:
                break
            calls = sub_calls.get(sub, [])
            if len(calls) != 1:
                refuse(f"Subscriber {sub} must be subscribed exactly once, found {len(calls)}")
                break
            args = calls[0]
            lit = _STRING_LIT.match(args[1].strip()) if len(args) >= 2 else None
            # Inputs may pass SubscriptionOptions, so the node can register
            # the deadline and message_lost handlers: BNA, the
            # stock policies since probe P20.
            if not args or _norm(args[0]) != "this" or lit is None or len(args) > 4:
                refuse(f"Subscriber {sub} must be subscribed as "
                       "subscribe(this, \"topic\"[, qos[, options]])")
                break
            sync_inputs.append(ScannedInput(
                member=sub, topic=lit.group(1), message_type=subscribers[sub],
                qos_args=args[2].strip() if len(args) >= 3 else None,
                sync_callback=cbs[0],
                callback=f"{cbs[0]}_via_{sub.strip('_')}",
                observer=observers.get(sub, [None])[0],
            ))
        else:
            admitted.append(MessageFiltersSyncInfo(
                member_variable=member, policy=policy, queue_size=queue,
                inputs=list(ins), callback=cbs[0], bna_params=bna_params,
                queue_param=queue_params.get(member) if queue is None else None,
            ))
            inputs.extend(sync_inputs)
    for sub in subscribers:
        if sub not in used:
            refuse(f"Subscriber {sub} does not feed a synchronizer "
                   "(standalone or chained filters are not supported)")
    for target in callbacks:
        if target not in syncs and target not in subscribers:
            refuse(f"registerCallback on {target} is not supported "
                   "(only on a recognised Synchronizer member)")

    return ScanResult(
        info=MessageFiltersInfo(synchronizers=admitted, unsupported=unsupported),
        inputs=inputs,
    )


_INT_LITERAL = re.compile(r"(0|[1-9][\d']*)([uU]?(?:ll|LL|l|L)?|(?:ll|LL|l|L)[uU])")


def _cpp_int_type(digits: int, suffix: str) -> tuple[int, bool] | None:
    """(bits, unsigned) of a decimal C++ literal on LP64, or None if no type holds it."""
    unsigned = "u" in suffix.lower()
    longs = suffix.lower().count("l")
    widths = [w for w, rank in ((32, 0), (64, 1), (64, 2)) if rank >= longs]
    for bits in widths:
        if digits <= (2**bits - 1 if unsigned else 2**(bits - 1) - 1):
            return bits, unsigned
    return None


def _literal_int(expr: str) -> int | None:
    """An integer literal or a product of them (``30 * 1000000``), else None.

    Evaluated with the C++ types of the operands (LP64, usual arithmetic
    conversions, left to right), so ``3000 * 1000000`` is None: it overflows
    ``int`` in C++ although the product fits ``int64_t``. Unsigned wraparound and
    octal literals are refused too, since the node would see another value.
    """
    expr = expr.strip()
    while expr.startswith("(") and expr.endswith(")"):
        expr = expr[1:-1].strip()
    value, kind = None, None
    for part in expr.split("*"):
        m = _INT_LITERAL.fullmatch(part.strip())
        if m is None:
            return None
        digits = int(m.group(1).replace("'", ""))
        t = _cpp_int_type(digits, m.group(2))
        if t is None:
            return None
        if value is None:
            value, kind = digits, t
            continue
        (b1, u1), (b2, u2) = kind, t
        bits = max(b1, b2)
        unsigned = (u1 and b1 == bits) or (u2 and b2 == bits)
        value *= digits
        kind = (bits, unsigned)
        if value > (2**bits - 1 if unsigned else 2**(bits - 1) - 1):
            return None
    return value


def bna_literal_params(policy_arg: str) -> dict[str, int] | None:
    """The seven BNA parameters of ``Policy(a, ..., g)``, or None.

    None also when the values fail the engine's own check (``Params::valid``),
    which would throw in the node constructor.
    """
    m = re.search(r"\(", policy_arg)
    got = _call_args_after(policy_arg, m.start()) if m else None
    if got is None or len(got[0]) != len(BNA_PARAM_NAMES):
        return None
    values = [_literal_int(a) for a in got[0]]
    if any(v is None or v > 2**63 - 1 for v in values):
        return None
    p = dict(zip(BNA_PARAM_NAMES, values))
    if p["capacity0"] < 1 or p["capacity1"] < 1 or p["retention"] <= 0 or p["timeout"] <= 0:
        return None
    return p


def qos_from_args(qos_args: str | None) -> dict[str, object]:
    """Map the third subscribe argument to SubscriberInfo QoS fields.

    Absent means ``rmw_qos_profile_default`` (reliable, volatile, keep_last 10).
    Unrecognised spellings leave every field None rather than guessing.
    """
    default = {"qos_depth": 10, "qos_reliability": "reliable",
               "qos_durability": "volatile", "qos_history": "keep_last"}
    if qos_args is None:
        return default
    a = _norm(qos_args)
    if a == "rmw_qos_profile_default":
        return default
    if a in ("rmw_qos_profile_sensor_data", "rclcpp::SensorDataQoS()",
             "rclcpp::SensorDataQoS().get_rmw_qos_profile()"):
        return {"qos_depth": 5, "qos_reliability": "best_effort",
                "qos_durability": "volatile", "qos_history": "keep_last"}
    m = re.fullmatch(r"rclcpp::QoS\((\d+)\)(?:\.get_rmw_qos_profile\(\))?", a)
    if m:
        return {**default, "qos_depth": int(m.group(1))}
    return {"qos_depth": None, "qos_reliability": None,
            "qos_durability": None, "qos_history": None}
