% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_function_to_task,
    [
        shape/2,
        find_categorical_shapes/2,  
        categorical_matching/2, 
        task_object/2,
        task_component/2,
        object_component/2,
        select_primitive/4
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/4. 


:- rdf_meta
    shape(+,r), 
    categorical_matching(+,r), 
    task_component(r,r), 
    task_object(r,r), 
    select_primitive(r,r,r,o),
    primitive_values(+,+,+,+).
    
% return all ID that match the condition
shape(ID, pap:'Cap') :-                                           % rules for a cap  
    primitive_values(ID, Eps, Dim, _), 
    hard_condition_max(Dim, [0.04,0.04,0.04]),            % x,y,z smaller than 4 cm 
    hard_condition_min(Dim, [0.01,0.01,0.01]),            % x,y,z greater than 1 cm
    max_shape_difference(2.2, Dim),                       % compare shape difference x, y, z with threshold (1.3)
    (shape_identification(Eps, cylindrical) ; 
        shape_identification(Eps, rectangular)).

shape(ID_final, pap:'Body') :-                                    % rules for a body
    findall(Dim,    
        (primitive_values(_, _, Dim, _), 
        hard_condition_min(Dim, [0.01,0.01,0.01]),         % x,y,z greater than 4 cm
        hard_condition_max(Dim, [0.15,0.15,0.15]),         % x,y,z smaller than 15 cm (small products)    
        soft_condition_min(Dim, 0.04),
        max_shape_difference(5, Dim)
     ), Dim), 

    % find biggest primitive
    flatten2D(Dim, Dim_flat),                       % flatten the list
    roundlist(Dim_flat, Dim_round),                 % roundoff to x digits
    % max_volume(Dim_round, ID_final).
    max_list(Dim_round, Max_value),                 % find max value
    primitive_values(ID_final, Eps, Dim1, _),       % iterate through primitives
    roundlist(Dim1, Dim_round1),                    % roundoff to x digits
    member(Max_value, Dim_round1),                  % find max value in primitive_values
    (shape_identification(Eps, cylindrical);
        shape_identification(Eps, rectangular)). 

shape(ID, pap:'Handle') :-                                    % rules for a handle
    primitive_values(ID, Eps, Dim, _),
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),
    hard_condition_max(Dim, [0.1, 0.1, 0.1]),
    soft_condition_min(Dim, 0.025),
    soft_condition_max(Dim, 0.025),                   % at least one shape is between certain parameters
    min_shape_difference(Dim, 2),                     % at least one axis is x times as long as the others
    shape_identification(Eps, cylindrical).             % handle must be of shap rectangular


% Assigns categories to shape primitives
find_categorical_shapes(ID, List) :- 
    primitive_values(ID, _, _, _),          % check all shape primitives
    findall(List, shape(ID, List), List).   % categorizes shape primitives
        
% Find IDs based on requested Shape
categorical_matching(ID, Shape) :- 
    findall(ID, (
        find_categorical_shapes(ID, List),  % find all ID's containing Shape
        memberchk(Shape, List)), ID).       % Check whether Shape is part of List

%%%%% TRIPLE - Reasoning %%%%%
task_object(Object,Task) :- triple(Object, _, D), 
                triple(D, _, pap:'hasTask'), 
                triple(D, _, Task),
                subclass_of(Task,pap:'Task').

task_component(Task,Component) :- triple(Task, _, D), 
                triple(D, _, pap:'hasComponent'), 
                triple(D, _, Component),
                subclass_of(Component, pap:'IndividualComponents').

object_component(Object, Component) :- 
                triple(Object, _, D), 
                triple(D, _, pap:'hasComponent'), 
                triple(D, _, Component),
                subclass_of(Component, pap:'IndividualComponents'),
                subclass_of(Object, pap:'Product').


select_primitive(Object, Task, ID, Component) :- 
    task_object(Object, Task),                                         % Check whether the task is possible.
    task_component(Task, Component),
    object_component(Object, Component),  
    categorical_matching(ID, Component).   


