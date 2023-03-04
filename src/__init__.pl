:- register_ros_package(knowrob).
:- register_ros_package(grasp_generator).

:- load_owl('package://grasp_generator/owl/affordance_ont.owl', [namespace(pap, 'http://www.semanticweb.org/stanz/ontologies/2022/11/affordance-ont#')]).

% Add prolog modules that you want to load when starting the prolog package
:- use_module(library('functions')).

% Reasoning Version-1
:- use_module(library('task_specific_rules')).

% Reasoning Version-2
% :- use_module(library('affordance_rules')).
