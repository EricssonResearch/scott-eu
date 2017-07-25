:- module(oslc_core, [
  post_resource/3
]).

:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_error)).

:- oslc_get((*):(*), handle_get, 0).
:- oslc_post((*):(*), handle_post, 0).
:- oslc_delete((*):(*), handle_delete, 0).

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
  catch((
    copy_resource(IRI, IRI, rdf, rdf(GraphOut), [inline(rdf)|Options])
  ),
    oslc_error(Message),
    format('Status: 400~n~n~w~n', [Message])
  ).

handle_post(_, IRI, GraphIn, _) :-
  post_resource(IRI, rdf(GraphIn), rdf(user)).

post_resource(IRI, Source, Sink) :-
  catch((
    once((
      rdf(IRI, rdf:type, oslc:'CreationFactory')
    ; oslc_error('Resource [~w] is not a creation factory', [IRI])
    )),
    once((
      oslc:unmarshal_property(NewResource, rdf:type, Class, _, Source),
      \+ rdf_is_bnode(NewResource)
    ; oslc_error('Resource type must be specified', [])
    )),
    once((
      \+ oslc:unmarshal_property(NewResource, _, _, _, Sink)
    ; oslc_error('Resource [~w] already exists', [NewResource])
    )),
    once((
      oslc_resource(IRI, [resourceShape=Shapes], rdf),
      member(Shape, Shapes),
      rdf(Shape, rdf:type, oslc:'ResourceShape'),
      oslc_resource(Shape, [describes=Classes], rdf),
      member(Class, Classes)
    ; oslc_error('Creation factory [~w] does not support type [~w]', [IRI, Class])
    )),
    copy_resource(NewResource, NewResource, Source, Sink, []),
    setting(oslc_prolog_server:base_uri, BaseUri),
    rdf_global_id(Prefix:Name, NewResource),
    atomic_list_concat([BaseUri, Prefix, '/', Name],  Location),
    format('Status: 201~nLocation: ~w~n~n', [Location])
  ),
    oslc_error(Message),
    format('Status: 400~n~n~w~n', [Message])
  ).

handle_delete(_, IRI) :-
  catch((
    delete_resource(IRI, rdf),
    format('Status: 200~n~n')
  ),
    oslc_error(Message),
    format('Status: 400~n~n~w~n', [Message])
  ).
