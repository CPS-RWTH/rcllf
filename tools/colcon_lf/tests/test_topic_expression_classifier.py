# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for ``clang_parser.classify_topic_expression``.

The topic-argument classifier is deliberately factored as a **pure function**
over already-extracted AST facts (cursor-kind names, string-literal count,
referenced variable names) so its decision table is testable on a bare host.
``tests/test_clang_parser.py`` needs real ROS2/rclcpp headers for libclang to
resolve node base classes and skips/fails without them; the AST-walking wrapper
``_classify_topic_argument`` is exercised there and in the dev container, but
the *logic* that decides "is this topic name statically resolvable?" is covered
here with no libclang, no ROS2, and no filesystem.
"""

from __future__ import annotations

from colcon_lf.node_analysis.clang_parser import classify_topic_expression


def _classify(kinds, literal_count, vars_=(), resolved_consts=0):
    return classify_topic_expression(
        list(kinds),
        literal_count=literal_count,
        referenced_var_names=list(vars_),
        resolved_const_count=resolved_consts,
    )


class TestStaticallyResolvable:
    def test_bare_string_literal(self):
        """create_publisher<T>("sensor_data", 10) — the common, resolvable case."""
        is_dynamic, reason = _classify(
            ["UNEXPOSED_EXPR", "STRING_LITERAL"], literal_count=1
        )
        assert is_dynamic is False
        assert reason == ""

    def test_literal_wrapped_in_std_string_construction(self):
        """A `const std::string &` parameter adds constructor/cast wrappers."""
        is_dynamic, _ = _classify(
            [
                "UNEXPOSED_EXPR",
                "CXX_CONSTRUCT_EXPR",
                "MATERIALIZE_TEMPORARY_EXPR",
                "CXX_BIND_TEMPORARY_EXPR",
                "UNEXPOSED_EXPR",
                "STRING_LITERAL",
            ],
            literal_count=1,
        )
        assert is_dynamic is False

    def test_resolvable_const_variable_is_not_dynamic(self):
        """`create_publisher<T>(kTopic, 10)` with `constexpr char kTopic[] = "x"`.

        The literal lives in the *declaration's* subtree, not the call's, so
        ``literal_count`` is 0 here; ``resolved_const_count`` carries the fact
        that ``_string_from_var_decl`` recovered the real name. Verified against
        real libclang; without this the constexpr-topic idiom
        (already covered by ``_first_string_literal``'s fallback and used across
        the example packages) would have produced a false RCLLF-H004.
        """
        is_dynamic, reason = _classify(
            ["UNEXPOSED_EXPR", "DECL_REF_EXPR"], literal_count=0, resolved_consts=1
        )
        assert is_dynamic is False
        assert reason == ""

    def test_parenthesized_literal(self):
        is_dynamic, _ = _classify(
            ["PAREN_EXPR", "UNEXPOSED_EXPR", "STRING_LITERAL"], literal_count=1
        )
        assert is_dynamic is False


class TestDynamicConcatenation:
    def test_two_string_literals_is_concatenation(self):
        """`"prefix_" + suffix_literal` — the silent partial-name case.

        Before Phase 7, ``_first_string_literal`` returned ``"prefix_"`` here and
        emitted **no** warning at all, so the analyzer recorded a wrong topic
        name that looked authoritative.
        """
        is_dynamic, reason = _classify(
            ["BINARY_OPERATOR", "STRING_LITERAL", "STRING_LITERAL"], literal_count=2
        )
        assert is_dynamic is True
        # Structural markers are reported ahead of the literal count because
        # the node kind names the shape of the computation.
        assert "BINARY_OPERATOR" in reason

    def test_two_literals_without_an_operator_node(self):
        """A helper call like `join("a", "b")` still composes two operands."""
        is_dynamic, reason = _classify(
            ["CALL_EXPR", "STRING_LITERAL", "STRING_LITERAL"], literal_count=2
        )
        assert is_dynamic is True
        assert "concatenation" in reason

    def test_two_resolved_consts_is_still_concatenation(self):
        """Two `constexpr` name fragments joined by a helper — both resolve, neither alone."""
        is_dynamic, reason = _classify(
            ["CALL_EXPR", "DECL_REF_EXPR", "DECL_REF_EXPR"],
            literal_count=0,
            resolved_consts=2,
        )
        assert is_dynamic is True
        assert "concatenation" in reason

    def test_literal_plus_std_to_string_call(self):
        """`"chatter_" + std::to_string(id_)` — dynamic_topic_example's publisher."""
        is_dynamic, reason = _classify(
            [
                "BINARY_OPERATOR",
                "UNEXPOSED_EXPR",
                "STRING_LITERAL",
                "CALL_EXPR",
                "DECL_REF_EXPR",
                "MEMBER_REF_EXPR",
            ],
            literal_count=1,
            vars_=["instance_id_"],
        )
        assert is_dynamic is True
        assert "BINARY_OPERATOR" in reason

    def test_literal_combined_with_unresolvable_variable_without_operator(self):
        """A helper call like `make_name(prefix_, "suffix")` still composes at runtime."""
        is_dynamic, reason = _classify(
            ["CALL_EXPR", "DECL_REF_EXPR", "UNEXPOSED_EXPR", "STRING_LITERAL"],
            literal_count=1,
            vars_=["prefix_"],
        )
        assert is_dynamic is True
        assert "prefix_" in reason


class TestDynamicNoLiteral:
    def test_bare_unresolvable_variable(self):
        is_dynamic, reason = _classify(
            ["UNEXPOSED_EXPR", "DECL_REF_EXPR"], literal_count=0, vars_=["topic_name"]
        )
        assert is_dynamic is True
        assert "topic_name" in reason

    def test_member_field_reference(self):
        is_dynamic, reason = _classify(
            ["UNEXPOSED_EXPR", "MEMBER_REF_EXPR", "CXX_THIS_EXPR"],
            literal_count=0,
            vars_=["topic_name_"],
        )
        assert is_dynamic is True
        assert "MEMBER_REF_EXPR" in reason

    def test_empty_expression_with_no_literal_and_no_variable(self):
        is_dynamic, reason = _classify([], literal_count=0)
        assert is_dynamic is True
        assert "no string literal" in reason

    def test_ternary_topic_choice(self):
        is_dynamic, reason = _classify(
            ["CONDITIONAL_OPERATOR", "STRING_LITERAL", "STRING_LITERAL"],
            literal_count=2,
        )
        assert is_dynamic is True
        assert "CONDITIONAL_OPERATOR" in reason

    def test_array_subscript_lookup(self):
        is_dynamic, reason = _classify(
            ["ARRAY_SUBSCRIPT_EXPR", "DECL_REF_EXPR", "INTEGER_LITERAL"],
            literal_count=0,
            vars_=["topics"],
        )
        assert is_dynamic is True
        assert "ARRAY_SUBSCRIPT_EXPR" in reason

    def test_unexpected_node_kind_is_reported_dynamic_conservatively(self):
        """An unrecognized node next to a literal fails safe (flagged, not trusted)."""
        is_dynamic, reason = _classify(
            ["UNEXPOSED_EXPR", "STRING_LITERAL", "SOME_FUTURE_CLANG_KIND"],
            literal_count=1,
        )
        assert is_dynamic is True
        assert "SOME_FUTURE_CLANG_KIND" in reason
