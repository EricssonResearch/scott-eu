:- module(oslc_error, [
  oslc_error/2
]).

% FIXME: probably there is a better way to report errors...

oslc_error(Message, Arguments) :-
  error0(Message, Arguments, []).

error0(Message, [], Accum):-
  reverse(Accum, Arguments),
  format(atom(Error), Message, Arguments),
  throw(oslc_error(Error)).

error0(Message, [H|T], Accum) :-
  to_local_name(H, H2),
  error0(Message, T, [H2|Accum]).

to_local_name(H, H2) :-
  \+ is_list(H),
  ( rdf_is_resource(H)
  -> rdf_global_id(H2, H)
  ; ( H = ^^(H2, _)
    -> true
    ; H2 = H
    )
  ).

to_local_name([], []) :- !.

to_local_name([H|T], [H2|T2]) :-
  to_local_name(H, H2),
  to_local_name(T, T2).
