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
  ( rdf_is_resource(H)
  -> rdf_global_id(H2, H)
  ; H2 = H
  ),
  error0(Message, T, [H2|Accum]).
