/*************************************************************************

    Copyright (C) 2020 Ericsson Research

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software Foundation, Inc., 
    59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*************************************************************************/
:- module(parseRDF,[parseRDF/2,
					first_atom_uppercase/2,
					find_text/2,
					find_sent/4,
					find_object/3,
					find_subject/3,
					find_predicate/4,
					assemble_sent/7,
					create_graph/1,
					assemble_turtle/6]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/turtle)).
:- rdf_register_prefix(pddl,'http://ontology.cf.ericsson.net/pddl/').
:- rdf_register_prefix(pddle,'http://ontology.cf.ericsson.net/pddl_example/.').

parseRDF([FOL], _):-
	find_text(FOL,_).
%%   	assemble_goal(TurtText,Turtgoal).

find_text(FOL,_):-
	FOL=.. [Cood|[FOL1,FOL2]],
	first_atom_uppercase(Cood,Goaltype),
	create_graph(Graph),
	find_sent(FOL1,_,Graph,Bnode1),
	find_sent(FOL2,_,Graph,Bnode2),
	assemble_turtle(Goaltype,Graph,_,Bnode1,Bnode2,_).


%% find_text(FOL,TurtText):-
%% 	FOL = and(FOL1,FOL2),
%%	Cood = "And",
%%	find_sent(FOL1,Sent1),
%%	find_sent(FOL2,Sent2).
%%	assemble_text(Cood,Sent1,Sent2,TurtText).

first_atom_uppercase(WordLC, AtomUC):-
    atom_chars(WordLC, [FirstChLow|LWordLC]),
    atom_chars(FirstLow, [FirstChLow]),
    upcase_atom(FirstLow, FirstUpp),
    atom_chars(FirstUpp, [FirstChUpp]),
    atom_chars(WordUC, [FirstChUpp|LWordLC]),
    atom_chars(WordUC, AtomUC).



find_text(FOL,_):-
    _ = [],
    find_sent(FOL,_,sent,bnode).

find_sent(FOL_NO_Cood,_,Graph,Bnode):-
	find_object(FOL_NO_Cood,Subject,FOL_No_OBJ),
	find_subject(FOL_No_OBJ,Object,FOl_No_SUB),
	find_predicate(FOl_No_SUB,Subpredicate1,Subpredicate2,Predicate),
	assemble_sent(Subject,Predicate,Subpredicate1,Subpredicate2,Object,Graph,Bnode).

find_object(FOL_NO_Cood,Subject,FOL_No_OBJ):-
	FOL_NO_Cood = some(_,X),
	X = and(Y,FOL_No_OBJ),
	Y =.. [Subject,_].
%%	format(Subject).

find_subject(FOL_No_OBJ,Object,FOl_No_SUB):-
	FOL_No_OBJ = some(_,X),
	X = and(Y,FOl_No_SUB),
	Y =.. [Object,_]. 
%%	format(Object).

find_predicate(FOl_No_SUB,Subpredicate1,Subpredicate2,Predicate):-
	FOl_No_SUB = location(X,_),
	X=.. [Predicate|_],
	term_to_atom(Predicate-x,Subpredicate1),
	term_to_atom(Predicate-y,Subpredicate2).


assemble_sent(Subject,Predicate,Subpredicate1,Subpredicate2,Object,Graph,Bnode):-
	rdf_create_bnode(Bnode),

	rdf_global_id(pddle:Predicate,Pred),
	rdf_global_id(pddle:Subpredicate1,Subpred1),
	rdf_global_id(pddle:Subpredicate2,Subpred2),
	rdf_global_id(pddle:Subject,Sub),
	rdf_global_id(pddle:Object,Obj),

	rdf_assert(Bnode,rdf:type,Pred,Graph),
	rdf_assert(Bnode,Subpred1,Sub,Graph),
	rdf_assert(Bnode,Subpred2,Obj,Graph).


create_graph(Graph):-
	rdf_retractall(_,_,_),
	atom_string(Graph,"text"),
	rdf_create_graph(Graph).
	%rdf_register_prefix(pddl,'http://ontology.cf.ericsson.net/pddl/'),
	%rdf_register_prefix(pddle,'http://ontology.cf.ericsson.net/pddl_example/.').




assemble_turtle(Goaltypechars,Graph,Turtletext,Bnode1,Bnode2,Fbnode):-
	rdf_create_bnode(Fbnode),
	%% Convert to Pddl:Or
	atom_chars(Goaltype,Goaltypechars),
	rdf_global_id(pddl:Goaltype, Pddlgoal),
	%% Convert type to Type
	%% Convert to Pddl:Type
%	atom_string(Pddlargument,"argument"),
	
	rdf_assert(Fbnode,pddl:argument,Bnode1,Graph),
	rdf_assert(Fbnode,pddl:argument,Bnode2,Graph),
	rdf_assert(Fbnode,rdf:type,Pddlgoal,Graph),
	rdf_save_turtle(Turtletext,[align_prefixes(true),prefixes([rdf,pddl,pddle])]).
