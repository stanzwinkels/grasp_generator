%% 
% ___init__.pl is consulted to load several rosprolog modules
% Makes the exported predicates available to the user
%%


% make sure library path is expanded
:- register_ros_package(knowrob).
:- register_ros_package(grasp_generator).

:- load_owl('package://grasp_generator/owl/primitive_ontol.owl', [namespace(pap, 'http://www.semanticweb.org/stanz/primitive-ontol#')]).

% Add prolog modules that you want to load when starting the prolog package
:- use_module(library('shape_to_task')).
:- use_module(library('shape_to_function_to_task')).
:- use_module(library('functions')).
% :- use_module(library('tasks')).

