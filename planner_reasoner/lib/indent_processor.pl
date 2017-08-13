:- module(indent_processor, [
  insert_indents/2
]).

insert_indents(In, Out) :-
  insert_indents(In, Out, 0, false).

insert_indents(In, Out, Indent, ApplyIndent) :-
  flatten(In, Flat),
  insert_indents0(Flat, Indented, Indent, ApplyIndent),
  flatten(Indented, Out).

insert_indents0([], [], _, _) :- !.

insert_indents0([indent(N, H)|T], [H2|T2], Indent, ApplyIndent) :- !,
  NewIndent is Indent + N,
  insert_indents(H, O, NewIndent, true),
  H2 = ["\n", O],
  insert_indents0(T, T2, Indent, ApplyIndent).

insert_indents0([H|T], [H2|T2], Indent, ApplyIndent) :-
  ( ApplyIndent == true
  -> format(string(H2), '~*|~w', [Indent, H])
  ; H2 = H
  ),
  ( string_concat(_, "\n", H)
  -> NewApplyIndent = true
  ; NewApplyIndent = false
  ),
  insert_indents0(T, T2, Indent, NewApplyIndent).
