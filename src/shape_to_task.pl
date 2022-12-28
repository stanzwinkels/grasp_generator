% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_task,
    [
        affordTask/3,
        primitive_values/5
    ]).



:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 


:- rdf_meta
    affordTask(+,r,r),
    primitive_values(+,+,+,+,+).


affordTask(ID, Surface, pap:'Cook') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    min_dimension(Dim_tool, [0.0, 0.08, 0.08]), 
    max_shape(ID_tool), 

    primitive_values(ID, _, Dim, _,Semantic_shape), 
    min_ratio(Dim, 1.5),   
    max_dimension(Dim, [0.05, 0.05, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    Semantic_shape == 'cylinder',
    ID \= ID_tool,
    Surface = 'Round'.

affordTask(ID, Surface, pap:'Hammer') :-
    primitive_values(ID_tool, _, _, _, _),
    % max_ratio(Dim_tool, 4), 

    primitive_values(ID, _, Dim, _, _),
    max_dimension(Dim, [0.07, 0.07, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    min_ratio(Dim, 1.5),   
    max_shape(ID), 
    ID_tool \= ID, 
    Surface = 'Round'.

affordTask(ID, Surface, pap:'Pour'):- 
    primitive_values(ID, _, Dim, _,Semantic_shape),  
    max_shape(ID),
    min_ratio(Dim, 1.5),
    max_dimension(Dim, [0.08, 0.08, 1.00]),
    min_dimension(Dim, [0.03, 0.03, 0.03]),
    Semantic_shape == 'cylinder',
    Surface = 'Round'. 
    
affordTask(ID, Surface, pap:'Scoop') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_ratio(Dim_tool, 2), 
    min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

    primitive_values(ID, _, Dim, _, _),  
    max_dimension(Dim, [0.005, 0.03, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    ID \= ID_tool, 
    Surface = 'All'.

affordTask(ID, Surface, pap:'Turn') :- 
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_dimension(Dim_tool, [0.03, 0.1, 0.1]),
    min_dimension(Dim_tool, [0.00, 0.03, 0.03]),

    primitive_values(ID, _, Dim, _, Semantic_shape),
    min_ratio(Dim, 2), 
    max_dimension(Dim, [0.04, 0.04, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    (Semantic_shape == 'cylinder';
        Semantic_shape == 'cuboid'),
    ID \= ID_tool, 
    Surface = 'All'. 


affordTask(ID, Surface, pap:'HandOver') :-
    primitive_values(ID, _, _, _,_),
    min_shape(ID),
    Surface = 'All'.


affordTask(ID, Surface, pap:'Placement') :-
    primitive_values(ID, _, _, _,_),
    max_shape(ID),
    Surface = 'Flat'.



    


