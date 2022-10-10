% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_task,
    [
        task_prim/2,
        primitive_values/4
    ]).


:- dynamic primitive_values/4. 

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_meta
    task_prim(+,r),
    primitive_values(+,+,+,+).


task_prim(ID, pap:'Bake') :-
    primitive_values(ID_pan, _, Dim_pan, _),
    hard_condition_min(Dim_pan, [0.04, 0.04, 0.04]),
    
    primitive_values(ID, Eps, Dim, _), 
    soft_condition_min(Dim, 0.04),
    min_shape_difference(Dim, 3),                
    shape_identification(Eps, cylindrical), 
    ID \= ID_pan.
    
    % 2 objects (Handle - Pan)
    % Handle: (Thin, Long, cylindrical)
    % Pan: (Big, Squared)


task_prim(ID, pap:'Hammer') :- 
    primitive_values(ID_tool, _, Dim_tool, _),
    max_shape_difference(Dim_tool, 4),

    primitive_values(ID, _, Dim, _),
    hard_condition_min(Dim, [0.005, 0.005, 0.005]),  
    max_shape(ID), 
    min_shape_difference(Dim, 2),
    ID_tool \= ID.

    % 2 objects (tool - handle) 
    % Tool: (rectangular, squared)
    % Handle: (Long, thin, cylindrical, bigger then tool),


task_prim(ID, pap:'Pour') :- 
    primitive_values(ID, Eps, Dim, _),  
    max_shape(ID), 
    min_shape_difference(Dim, 2),
    hard_condition_min(Dim, [0.02, 0.2, 0.02]),
    soft_condition_min(Dim, 0.04), 
    shape_identification(Eps, cylindrical). 
    % Different objects (Mug - Bottle)
    % Bottle consists of 2 parts (cap - body)
        % cap (small, cubical)
        % body (long, biggest)
    % Mug consists of 2 parts (handle - body)
        % handle (small, cubical)
        % body (biggest, cubical)

task_prim(ID, pap:'Handover') :-
    primitive_values(ID, _, _, _),
    min_shape(ID).
    
    % Always prefers the smallest part for grasping. 


% environment(ID, 'Shelf') :- 
    % prefer a grasp at the top. 
    % for a hammer grasp at the handle. 





% task_prim(ID, pap:'Scoop')

% task(Task, 'Scoop')
%     primitive_values(ID, EPS, Shape, Pos),
%     hard_condition_min(Shape, [0.01, 0.01, 0.01]),
%     hard_condition_max(Shape, [0.1, 0.1, 0.1]),
%     soft_condition_min(Shape, 0.025),
%     soft_condition_max(Shape, 0.025),    
%     shape_identification(EPS, rectangular),

%     primitive_values(ID, EPS, Shape, Pos),
%     shape_identification(EPS, spherical),
%     max_shape_difference(1.5, Shape_body),

%     % grasp the long pointy handle
%     % A scoup contains usually a cylindrical type of object. 
%     % Both components are parallel, one dimensions is almost zero.

% task(Task, 'Wrench')
%     % A thin object with a nudge. 
%     %  Grasp it in the middle. 
