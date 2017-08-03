:- module(oslc_core, [
  post_resource/3
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(oslc)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_error)).

:- oslc_get((*):'', handle_ontology, 1).
:- oslc_get((*):(*), handle_get, 0).
:- oslc_post((*):(*), handle_post, 0).
:- oslc_put((*):(*), handle_put, 0).
:- oslc_delete((*):(*), handle_delete, 0).

:- oslc_get(oslc:time, get_current_time).
:- oslc_get(oslc:'Time', get_time_class).

get_current_time(Context) :-
  get_time(T),
  make_temp_graph(Context.graph_out),
  create_resource(oslc:time, [oslc:'Time'],
                 [dcterms:created='^^'(T, xsd:dateTime)], rdf(Context.graph_out)).

get_time_class(Context) :-
  make_temp_graph(Context.graph_out),
  create_resource(oslc:'Time', [rdfs:'Class'], [], rdf(Context.graph_out)).

handle_ontology(Context) :-
  once((
    rdf_graph(Graph),
    atom_concat(Graph, _, Context.iri)
  )),
  Context.graph_out = Graph.

handle_get(Context) :-
  IRI = Context.iri,
  once(rdf(IRI, _, _)),
  once((
    member(search(Search), Context.request),
    findall(Option, (
        member(Key=Value, Search),
        atom_concat('oslc.', OP, Key),
        Option =.. [OP, Value]
      ), Options
    )
  ; Options = []
  )),
  make_temp_graph(Context.graph_out),
  catch((
    copy_resource(IRI, IRI, rdf, rdf(Context.graph_out), [inline(rdf)|Options])
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request (problem with Options)
  ).

handle_post(Context) :-
  post_resource(Context.iri, rdf(Context.graph_in), rdf(user)).

post_resource(IRI, Source, Sink) :-
  catch((
    once((
      rdf(IRI, rdf:type, oslc:'CreationFactory')
    ; oslc_error('Resource [~w] is not a creation factory', [IRI])
    )),
    once((
      ground(Source)
    ; oslc_error('Missing resource in POST request to [~w]', [IRI])
    )),
    once((
      oslc:unmarshal_property(NewResource, rdf:type, Class, _, Source),
      \+ rdf_is_bnode(NewResource)
    ; oslc_error('Missing resource type in POST request to [~w]', [IRI])
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
    response(201, ['Location'(Location)]) % created
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).

handle_put(Context) :-
  catch((
    once((
      member(if_match(IfMatch), Context.request),
      atomic_list_concat([_, ReceivedHash, _], '\"', IfMatch)
    ; oslc_error('Missing or wrong header [If-Match] in PUT request to [~w]', [Context.iri_spec])
    )),
    IRI = Context.iri,
    autodetect_resource_graph(IRI, Graph),
    once((
      resource_md5(IRI, Graph, ReceivedHash),
      copy_resource(IRI, IRI, rdf(Context.graph_in), rdf(Graph), []),
      response(204) % no content
    ; format(atom(Message), 'The value of [If-Match] header does not match [~w]', [Context.iri_spec]),
      throw(response(412, Message)) % precondition failed
    ))
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).

handle_delete(Context) :-
  catch((
    autodetect_resource_graph(Context.iri, Graph),
    delete_resource(Context.iri, rdf(Graph)),
    response(204) % no content
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).
