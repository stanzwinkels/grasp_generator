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
    min_shape_difference
    min_shape_difference(Dim, 2),
    hard_condition_min(Dim, [0.03, 0.03, 0.03]),
    soft_condition_max(Dim, 0.05),      % there is at least on that is smaller than 7 cm
    soft_condition_min(Dim, 0.06),      % ther is at least on that is bigger
    shape_identification(Eps, cylindrical),
    Surface = 'Round'. 
    
    % Requires to be the largest shape of all as well.... 

    % Requires a cylindrically shaped object of certain dimension
    % Nothing above or under the main cylindrical




task_prim(ID, Surface, pap:'Hammer') :-
    primitive_values(ID_tool, _, Dim_tool, _),
    max_shape_difference(Dim_tool, 4),
    primitive_values(ID, _, Dim, _),
    hard_condition_min(Dim, [0.05, 0.05, 0.05]),  
    max_shape(ID), 
    min_shape_difference(Dim, 2),
    ID_tool \= ID.


task_prim(ID, Surface, pap:'Placement') :-
    primitive_values(ID, Eps, Dim, _),
    max_shape(ID),
    Surface = 'Flat'.

        

% % Here we create rules of what we expect the object to have for features. 
% task_prim(ID, pap:'Cook') :-
%     primitive_values(ID_pan, Eps_pan, Dim_pan, _),
%     hard_condition_min(Dim_pan, [0.04, 0.04, 0.04]),
%     soft_condition_min(Dim_pan, 0.10),
%     min_shape_difference(Dim, 2),
%     shape_identification(Eps_pan, cylindrical), 

%     primitive_values(ID, Eps, Dim, _), 
%     soft_condition_min(Dim, 0.04),
%     min_shape_difference(Dim, 2),                
%     shape_identification(Eps, cylindrical), 
%     ID \= ID_pan.

%     % volume(ID > ID_pan) volume pan bigger than handle!
%     % Requires a pan shaped object
%     % cooking pan and baking pan. 


% task_prim(ID, pap:'PourIn') :-
%     primitive_values(ID, Eps, Dim, _),  
%     max_shape(ID), 
%     min_shape_difference(Dim, 2),
%     hard_condition_min(Dim, [0.02, 0.02, 0.02]),
%     soft_condition_min(Dim, 0.04), 
%     shape_identification(Eps, cylindrical). 
%     % Requires to be the largest shape of all as well.... 

%     % Requires a cylindrically shaped object of certain dimension
%     % Nothing above or under the main cylindrical

% task_prim(ID, pap:'PourOut') :-
%     primitive_values(ID, Eps, Dim, _),  
%     max_shape(ID), 
%     min_shape_difference(Dim, 2),
%     hard_condition_min(Dim, [0.02, 0.02, 0.02]),
%     soft_condition_min(Dim, 0.04), 
%     shape_identification(Eps, cylindrical). 
%     % Requires cylindrical shaped object

%     % no smaller component on top.... 
    


% task_prim(ID, pap:'Scoop') :-
%     primitive_values(ID_tool, _, Dim_tool, _),
%     max_shape_difference(Dim_tool, 1.5),
%     hard_condition_min(Dim, [0.02, 0.02, 0.02]),
%     %     shape_identification(Eps, spherical),
%     primitive_values(ID, _, Dim, _),  
%     min_shape_difference(Dim, 4),
%     ID \= ID_tool.
%     % Spherically shaped object
%     % Requires a handle as well


% task_prim(ID, pap:'Hammer') :-
%     primitive_values(ID_tool, _, Dim_tool, _),
%     max_shape_difference(Dim_tool, 4),
%     primitive_values(ID, _, Dim, _),
%     hard_condition_min(Dim, [0.05, 0.05, 0.05]),  
%     max_shape(ID), 
%     min_shape_difference(Dim, 2),
%     ID_tool \= ID.
    % Head and handle


% task_prim(ID, pap:'HandOver') :-
%     primitive_values(ID_tool, _, Dim_tool, _).
%     % Grasp anything except the big component. 



% % The advantage is that we can further specify certain rules for more specific scenario's
% % Compared to affodance, it would require a different setup to create such a rules. 

% task_prim(ID, pap:'Cook a salmon') :-
%     task_prim(ID, pap:'Cook'),
%     % additional constraint

% task_prim(ID, pap:'Cook an egg') :-


% task_prim(ID, pap:'PourIn a liter') :-