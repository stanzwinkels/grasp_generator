% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_function_to_task,
    [
        task_object/2,
        not_affordAffordance/2,
        affordAffordance/3,
        grasp_affordance/3,
        primitive_values/5
        % feasible_tasks/1,    
        % find_categorical_shapes/2,  
        % categorical_matching/2, 
        % task_component/2,
        % object_component/2,
        % select_primitive/4
        ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 


:- rdf_meta
    affordAffordance(+,r,r), 
    task_object(r,r),
    not_affordAffordance(r,r),
    grasp_affordance(r,r,r),
    primitive_values(+,+,+,+,+).
    % select_primitive(r,r,r,o),
    % categorical_matching(+,r), 
    % task_component(r,r), 
    % feasible_tasks(r),  


affordAffordance(ID, Surface, pap:'AffordGrasp') :-                 
    primitive_values(ID, _, Dim, _,Semantic_shape),
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Otherwise too small for a grasp component. 
    soft_condition_min(Dim, 0.05),
    max_dimension(Dim, [0.05, 0.05, 1]),
    min_ratio(Dim, 2),                       % at least one axis is x times as long as the others
    (Semantic_shape == 'cylinder' ;                     % Can be both rectangular and spherical shaped. 
        Semantic_shape == 'cuboid'),
    Surface = 'All'.

affordAffordance(ID, Surface, pap:'AffordWgrasp') :-                 
    primitive_values(ID, _, Dim, _, Semantic_shape),
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Otherwise too small for a grasp component.
    min_dimension(Dim, [0.05, 0.05, 0.05]), 
    min_ratio(Dim, 2),                       % at least one axis is x times as long as the others
    Semantic_shape == 'cylinder', 
    Surface = 'Round'. 

    
affordAffordance(ID, Surface, pap:'AffordContain') :-
    primitive_values(ID, _, Dim, _, Semantic_shape),
    Semantic_shape == 'cylinder',  
    hard_condition_min(Dim, [0.03,0.03,0.03]),          % Otherwise too small to be considered as a container
    max_shape(ID),
    Surface = 'Round'.   


affordAffordance(ID, Surface, pap:'AffordPound') :-
    primitive_values(ID, _, Dim, _, Semantic_shape), 
    min_dimension(Dim, [0.015, 0.015, 0.015]), 
    (Semantic_shape == 'cylinder' ;                     % Can be both rectangular and spherical shaped. 
        Semantic_shape == 'cuboid'),
    Surface = 'All'.       


affordAffordance(ID, Surface, pap:'AffordSupport') :-                            
    primitive_values(ID, _, Dim, _, _), 
    soft_condition_max(Dim, 0.03),                      % Thin surface for flipping    
    min_dimension(Dim, [0.0, 0.05, 0.05]),
    min_ratio(Dim, 3),
    Surface = 'All'.                       % Big difference LxWxH, one side is much thinner than the other lengths. 


affordAffordance(ID, Surface, pap:'AffordScoop') :-                 
    primitive_values(ID, _, Dim, _, Semantic_shape),                
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Must be larger than 1 cm, otherwise difficult to scoop. 
    hard_condition_max(Dim, [0.1, 0.1, 0.1]),           % Otherwise it is too big for a scoop and more of a container. 
    max_ratio(Dim, 2),                       % A scoop is spherical, and therefore 2 sides have similar lengths. 
    Semantic_shape == 'sphere',                          % A shape is spherically shaped. 
    Surface = 'All'.

not_affordAffordance(Affordance, Task) :- 
    disjoint_with(Task, Description),
    triple(Description, _, Affordance),
    subclass_of(Affordance, pap:'Affordance'). 

task_object(Affordance, Task) :- 
    subclass_of(Task, D), 
    triple(D, _, Affordance), 
    subclass_of(Affordance, pap:'Affordance'), 
    triple(D, _, pap:'RequiresAffordance').  
    
grasp_affordance(ID, Surface, Task) :-
    affordAffordance(ID, Surface, Affordance), 
    subclass_of(Task, Description1), 
    triple(Description1, _, Affordance), 
    subclass_of(Affordance, pap:'Affordance'), 
    triple(Description1,_, pap:'hasGraspAffordance').