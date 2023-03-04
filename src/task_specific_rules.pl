% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(task_specific_rules,
    [
        affordTask/2,
        primitive_values/5
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 

:- rdf_meta
    affordTask(+,r,r),
    primitive_values(+,+,+,+,+).


affordTask(ID, pap:'Cooking') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    min_dimension(Dim_tool, [0.0, 0.08, 0.08]), 
    max_shape(ID_tool), 

    primitive_values(ID, _, Dim, _,Semantic_shape), 
    min_ratio(Dim, 1.5),   
    max_dimension(Dim, [0.05, 0.05, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    Semantic_shape == 'cylinder',
    ID \= ID_tool.

affordTask(ID, pap:'Hammering') :-
    primitive_values(ID_tool, _, Dim_tool, _, _),
    max_ratio(Dim_tool, 4), 

    primitive_values(ID, _, Dim, _, _),
    max_dimension(Dim, [0.07, 0.07, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    min_ratio(Dim, 1.5),   
    max_shape(ID), 
    ID_tool \= ID.

affordTask(ID, pap:'Pouring'):- 
    primitive_values(ID, _, Dim, _,Semantic_shape),  
    max_shape(ID),
    min_ratio(Dim, 1.5),
    max_dimension(Dim, [0.08, 0.08, 1.00]),
    min_dimension(Dim, [0.03, 0.03, 0.03]),
    Semantic_shape == 'cylinder'.

affordTask(ID, pap:'Scooping') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_ratio(Dim_tool, 2), 
    min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

    primitive_values(ID, _, Dim, _, _),  
    max_dimension(Dim, [0.005, 0.03, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    ID \= ID_tool.

affordTask(ID, pap:'Turning') :- 
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_dimension(Dim_tool, [0.03, 0.1, 0.1]),
    min_dimension(Dim_tool, [0.00, 0.03, 0.03]),

    primitive_values(ID, _, Dim, _, Semantic_shape),
    min_ratio(Dim, 2), 
    max_dimension(Dim, [0.04, 0.04, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    (Semantic_shape == 'cylinder';
        Semantic_shape == 'cuboid'),
    ID \= ID_tool.


affordTask(ID, pap:'Handover') :-
    primitive_values(ID, _, _, _,_),
    min_shape(ID).
    