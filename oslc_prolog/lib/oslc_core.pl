:- module(oslc_core, []).

:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).

:- oslc_get((*):(*), handle_get, 0).
:- oslc_post((*):(*), handle_post, 0).

handle_get(Request, IRI, GraphOut) :-
  once(rdf(IRI, _, _)),
  once((
    member(search(Search), Request),
    findall(Option, (
        member(Key=Value, Search),
        atom_concat('oslc.', OP, Key),
        Option =.. [OP, Value]
      ), Options
    )
  ; Options = []
  )),
  once((
    copy_resource(IRI, IRI, rdf, rdf(GraphOut), [inline(rdf)|Options])
  ; format('Status: 400~n~n')
  )).

handle_post(_Request, _IRI, _GraphIn, _GraphOut) :-
  true.
