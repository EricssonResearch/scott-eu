:- module(oslc_cliopatria_ui, []).

:- http_handler(root(oslc_resource_shapes), oslc_resource_shapes, []).
:- http_handler(root(oslc_shape_resources), oslc_shape_resources, []).
:- http_handler(root(oslc_shape), oslc_shape, []).

:- include(bundle(html_page)).
:- use_module(components(label)).

cliopatria:menu_item(300=places/oslc_resource_shapes, 'Shapes').

%--------------- SHAPES

oslc_resource_shapes(_Request) :-
  findall(S, rdf(S, rdf:type, oslc:'ResourceShape'), Shapes),
  sort(Shapes, SortedShapes),
  reply_html_page(cliopatria(oslc),
  	title('OSLC'),
  	div([
      h1('OSLC Resource Shapes'),
      \shape_table(SortedShapes)
    ])).

shape_table(Shapes) -->
  html(table(class(block), [
    tr([
		  th('Resource Shape'),
      th('Description'),
      th('Properties'),
      th('Mandatory Properties'),
      th('Resources')
		])
  | \shape_rows(Shapes, 1)
  ])).

shape_rows([], _) --> [].
shape_rows([H|T], Row) -->
  odd_even_row(Row, Next, \shape_row(H)),
  shape_rows(T, Next).

shape_row(Shape) -->
  {
    once((
      rdf(Shape, dcterms:description, ShapeDescription)
    ; ShapeDescription = ''
    )),
    aggregate_all(count, rdf(Shape, oslc:property, _), PropertyCout),
    http_link_to_id(oslc_shape, [s=Shape], ShapeURI),
    findall(Mandatory, (
      rdf(Shape, oslc:property, Property),
      rdf(Property, oslc:occurs, Occurs),
      rdf_global_id(oslc:'Exactly-one', EO),
      rdf_global_id(oslc:'One-or-many', OM),
      member(Occurs, [EO, OM]),
      rdf(Property, oslc:propertyDefinition, PropertyDefinition),
      Mandatory = div(\rdf_link(PropertyDefinition))
    ), Mandatories),
    aggregate_all(count, rdf(_, oslc:instanceShape, Shape), ResourceCount),
    http_link_to_id(oslc_shape_resources, [s=Shape], ShapeResourcesURI)
  },
	html([
    td(\rdf_link(Shape)),
    td(style('max-width: 500px'), ShapeDescription),
    td(style('text-align:center'), a(href(ShapeURI), PropertyCout)),
    td(Mandatories),
    td(style('text-align:center'), a(href(ShapeResourcesURI), ResourceCount))
  ]).

%--------------- RESOURCES DESCRIBED BY SHAPE

oslc_shape_resources(Request) :-
  http_parameters(Request, [ s(Shape, []) ]),
  findall(R, rdf(R, oslc:instanceShape, Shape), Resources),
  sort(Resources, SortedResources),
  reply_html_page(cliopatria(oslc),
  	title('OSLC'),
  	div([
      h1(['OSLC Resources described by ', \rdf_link(Shape)]),
      \resource_table(SortedResources)
    ])).

resource_table(Resources) -->
  html(table(class(block), [
    tr([
		  th('Resource'),
      th('Property'),
      th('Value')
		])
  | \resource_rows(Resources, 1)
  ])).

resource_rows([], _) --> [].
resource_rows([H|T], Raw) -->
  odd_even(Raw, Next, Class),
  resource_row(H, Class),
  resource_rows(T, Next).

resource_row(Resource, Class) -->
  {
    findall(Key, rdf(Resource, Key, _), Keys),
    rdf_global_id(LResource, Resource),
    term_to_atom(LResource, ResourceLabel),
    http_link_to_id(list_resource, [r=Resource], ResourceURI),
    sort(Keys, SortedKeys),
    length(Keys, NumOfKeys),
    Span is NumOfKeys + 1,
    SortedKeys = [H|T]
  },
	html([
    tr(class(Class), [
      td(rowspan(Span), a(href(ResourceURI), ResourceLabel)),
      \resource_property(Resource, H, odd)
    ]),
    \resource_property_rows(Resource, T, 0),
    tr([
      td(style('border-width: 0 1px 1px 1px'), ''),
      td(style('border-width: 0 1px 1px 1px'), ''),
      td(style('border-width: 0 1px 1px 1px'), '')
    ])
  ]).

resource_property_rows(_, [], _) --> [].
resource_property_rows(Resource, [H|T], Row) -->
  odd_even(Row, Next, Class),
  resource_property(Resource, H, Class),
  resource_property_rows(Resource, T, Next).

resource_property(Resource, P, Class) -->
  {
    findall(Property, rdf(Resource, P, Property), Properties),
    length(Properties, NumOfProperties),
    Properties = [H|T]
  },
  html([
    tr(class(Class), [
      td([rowspan(NumOfProperties), style('text-align:right')], \rdf_link(P)),
      td(\rdf_link(H))
    ]),
    \property_value_rows(T, Class)
  ]).

property_value_rows([], _) --> [].
property_value_rows([H|T], Class) -->
  html(tr(class(Class), [td(\rdf_link(H))])),
  property_value_rows(T, Class).

odd_even(Row, Next, Class) -->
	{ (   Row mod 2 =:= 0
	  ->  Class = even
	  ;   Class = odd
	  ),
	  Next is Row+1
	}.

%--------------- SHAPE DEFINITION

oslc_shape(Request) :-
  http_parameters(Request, [ s(Shape, []) ]),
  findall(P, rdf(Shape, oslc:property, P), Properties),
  once((
    rdf(Shape, dcterms:description, ShapeDescription)
  ; ShapeDescription = ''
  )),
  sort(Properties, SortedProperties),
  findall(div(\rdf_link(T)), rdf(Shape, oslc:describes, T), Types),
  reply_html_page(cliopatria(oslc),
  	title('OSLC'),
  	div([
      h1(['OSLC Shape ', \rdf_link(Shape)]),
      div(ShapeDescription),
      \shape_property_table(SortedProperties),
      div(style('padding: 0.5cm 0 0 0'), 'Describes types: '),
      div(style('padding: 0.3cm 0 0 1cm'), Types)
    ])).

shape_property_table(Properties) -->
  html(table(class(block), [
    tr([
      th('Property'),
      th('Description'),
      th('Cardinality'),
      th('Value Type'),
      th('Representation'),
      th('Range')
		])
  | \shape_property_rows(Properties, 1)
  ])).

shape_property_rows([], _) --> [].
shape_property_rows([H|T], Row) -->
  odd_even_row(Row, Next, \shape_property_row(H)),
  shape_property_rows(T, Next).

shape_property_row(Property) -->
  {
    rdf(Property, oslc:propertyDefinition, PropertyDefinition),
    once((
      rdf(Property, dcterms:description, PropertyDescription)
    ; PropertyDescription = ''
    )),
    rdf(Property, oslc:occurs, Cardinality),
    once((
      rdf(Property, oslc:valueType, VT),
      ValueType = div(\rdf_link(VT))
    ; ValueType = ''
    )),
    once((
      rdf(Property, oslc:representation, Rep),
      Representation = div(\rdf_link(Rep))
    ; Representation = ''
    )),
    once((
      rdf(Property, oslc:range, Ran),
      Range = div(\rdf_link(Ran))
    ; Range = ''
    ))
  },
  html([
    td(\rdf_link(PropertyDefinition)),
    td(style('max-width: 500px'), PropertyDescription),
    td(\rdf_link(Cardinality)),
    td(ValueType),
    td(Representation),
    td(Range)
  ]).
