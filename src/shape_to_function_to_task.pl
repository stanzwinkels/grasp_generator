% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_function_to_task,
    [
        affordance/3,
        primitive_values/5        
        % find_categorical_shapes/2,  
        % categorical_matching/2, 
        % task_object/2,
        % task_component/2,
        % object_component/2,
        % select_primitive/4
        ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 


:- rdf_meta
    affordance(+,r,r), 
    % categorical_matching(+,r), 
    % task_component(r,r), 
    % task_object(r,r), 
    % select_primitive(r,r,r,o),
    primitive_values(+,+,+,+,+).


affordance(ID, Surface, pap:'Grasp') :-                 
    primitive_values(ID, _, Dim, _,Semantic_shape),
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Otherwise too small for a grasp component. 
    soft_condition_min(Dim, 0.05),
    soft_condition_max(Dim, 0.05),                      % at least one shape is between certain parameters
    min_shape_difference(Dim, 2),                       % at least one axis is x times as long as the others
    Semantic_shape == 'cylinder', 
    Surface = 'All'.

affordance(ID, Surface, pap:'Wgrasp') :-                 
    primitive_values(ID, _, Dim, _, Semantic_shape),
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Otherwise too small for a grasp component. 
    soft_condition_min(Dim, 0.05),                      % Both diameters should be at least 0.05m
    min_shape_difference(Dim, 2),                       % at least one axis is x times as long as the others
    Semantic_shape == 'cylinder', 
    Surface = 'Round'. 

    
affordance(ID, Surface, pap:'Contain') :-
    primitive_values(ID, _, Dim, _, Semantic_shape),
    Semantic_shape == 'cylinder',  
    hard_condition_min(Dim, [0.03,0.03,0.03]),          % Otherwise too small to be considered as a container
    max_shape(ID),
    Surface = 'Round'.   


affordance(ID, Surface, pap:'Pound') :-
    primitive_values(ID, _, Dim, _, Semantic_shape), 
    max_shape_difference(2.2, Dim),                     % Objects to pound with are typically cuboid 
    hard_condition_min(Dim, [0.03,0.03,0.03]),          % Otherwise too small to pound with
    (Semantic_shape == 'cylinder' ;                     % Can be both rectangular and spherical shaped. 
        Semantic_shape == 'cuboid'),
    Surface = 'All'.       


affordance(ID, Surface, pap:'Support') :-                            
    primitive_values(ID, _, Dim, _, _), 
    soft_condition_max(Dim, 0.01),                      % Thin surface for flipping
    min_shape_difference(3, Dim),
    Surface = 'All'.                       % Big difference LxWxH, one side is much thinner than the other lengths. 


affordance(ID, Surface, pap:'Scoop') :-                 
    primitive_values(ID, _, Dim, _, Semantic_shape),                
    hard_condition_min(Dim, [0.01, 0.01, 0.01]),        % Must be larger than 1 cm, otherwise difficult to scoop. 
    hard_condition_max(Dim, [0.1, 0.1, 0.1]),           % Otherwise it is too big for a scoop and more of a container. 
    max_shape_difference(2, Dim),                       % A scoop is spherical, and therefore 2 sides have similar lengths. 
    Semantic_shape == 'sphere',                          % A shape is spherically shaped. 
    Surface = 'All'.


% % Assigns categories to shape primitives
% find_categorical_shapes(ID, List) :- 
%     primitive_values(ID, _, _, _, _),          % check all shape primitives
%     findall(List, shape(ID, List), List).   % categorizes shape primitives
        
% % Find IDs based on requested Shape
% categorical_matching(ID, Shape) :- 
%     findall(ID, (
%         find_categorical_shapes(ID, List),  % find all ID's containing Shape
%         memberchk(Shape, List)), ID).       % Check whether Shape is part of List

% %%%%% TRIPLE - Reasoning %%%%%
% task_object(Object,Task) :- triple(Object, _, D), 
%                 triple(D, _, pap:'hasTask'), 
%                 triple(D, _, Task),
%                 subclass_of(Task,pap:'Task').

% task_component(Task,Component) :- triple(Task, _, D), 
%                 triple(D, _, pap:'hasComponent'), 
%                 triple(D, _, Component),
%                 subclass_of(Component, pap:'IndividualComponents').

% object_component(Object, Component) :- 
%                 triple(Object, _, D), 
%                 triple(D, _, pap:'hasComponent'), 
%                 triple(D, _, Component),
%                 subclass_of(Component, pap:'IndividualComponents'),
%                 subclass_of(Object, pap:'Product').


% select_primitive(Object, Task, ID, Component) :- 
%     task_object(Object, Task),                                         % Check whether the task is possible.
%     task_component(Task, Component),
%     object_component(Object, Component),  
%     categorical_matching(ID, Component).   


