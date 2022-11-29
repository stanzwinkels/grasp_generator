% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_task,
    [
        task_prim/3,
        primitive_values/4
    ]).


:- dynamic primitive_values/4. 

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_meta
    task_prim(+,r,r),
    primitive_values(+,+,+,+).


task_prim(ID, Surface, pap:'Cook') :-
    primitive_values(ID_pan, Eps_pan, Dim_pan, _),
    hard_condition_min(Dim_pan, [0.02, 0.02, 0.02]),
    soft_condition_min(Dim_pan, 0.10),
    shape_identification(Eps_pan, cylindrical), 
    max_shape_difference(Dim_pan, 2),

    primitive_values(ID, Eps, Dim, _), 
    min_shape_difference(Dim, 1.5),                
    soft_condition_min(Dim, 0.04),
    shape_identification(Eps, cylindrical), 
    ID \= ID_pan,
    Surface = 'Round'.

task_prim(ID, Surface, pap:'Pour'):- 
    primitive_values(ID, Eps, Dim, _),  
    max_shape(ID),
    min_shape_difference(Dim, 2),
    hard_condition_min(Dim, [0.03, 0.03, 0.03]),
    soft_condition_max(Dim, 0.05),      % there is at least on that is smaller than 7 cm
    soft_condition_min(Dim, 0.06),      % there is at least on that is bigger
    shape_identification(Eps, cylindrical),
    Surface = 'Round'. 
    
task_prim(ID, Surface, pap:'Hammer') :-
    primitive_values(ID_tool, _, Dim_tool, _),
    max_shape_difference(Dim_tool, 4),
    
    primitive_values(ID, _, Dim, _),
    hard_condition_min(Dim, [0.02, 0.02, 0.02]),
    min_shape_difference(Dim, 2),  
    max_shape(ID), 
    ID_tool \= ID, 
    Surface = "All".
    
task_prim(ID, Surface, pap:'Placement') :-
    primitive_values(ID, _, _, _),
    max_shape(ID),
    Surface = 'Flat'.

    
task_prim(ID, Surface, pap:'Scoop') :-
    primitive_values(ID_tool, _, Dim_tool, _),
    max_shape_difference(Dim_tool, 2),
    hard_condition_min(Dim, [0.02, 0.02, 0.02]),
    soft_condition_min(Dim, 0.05),
    %     shape_identification(Eps, spherical),
    primitive_values(ID, _, Dim, _),  
    min_shape_difference(Dim, 4),
    ID \= ID_tool, 
    Surface = 'All'.

task_prim(ID, Surface, pap:'HandOver') :-
    primitive_values(ID, _, _, _),
    min_shape(ID),
    Surface = 'All'.


task_prim(ID, Surface, pap:'Turn') :- 
    primitive_values(ID_tool, _, Dim_tool, _),
    soft_condition_max(Dim_tool, 0.01),
    soft_condition_min(Dim_tool, 0.05), 

    primitive_values(ID, _, Dim, _),
    min_shape_difference(Dim, 4),
    ID \= ID_tool, 
    Surface = 'All'. 
    


