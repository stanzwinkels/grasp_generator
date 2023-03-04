% module definition and public (externally visible)roslau predicates
:- module(affordance_rules,
    [
        task_object/2,
        grasp_affordance/2,
        partAffordance/2,
        primitive_values/5
        ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 

:- rdf_meta
    partAffordance(+,r), 
    task_object(r,r),
    grasp_affordance(r,r),
    primitive_values(+,+,+,+,+).

partAffordance(ID, pap:'AffordGrasp') :-                 
    primitive_values(ID, _, Dim, _,Semantic_shape),
    min_dimension(Dim, [0.001, 0.01, 0.04]),
    max_dimension(Dim, [0.05, 0.05, 1]),
    min_ratio(Dim, 2),                      
    (Semantic_shape == 'cylinder' ;                     
        Semantic_shape == 'cuboid').

partAffordance(ID, pap:'AffordWgrasp') :-                 
    primitive_values(ID, _, Dim, _, Semantic_shape),
    min_dimension(Dim, [0.05, 0.05, 0.05]), 
    min_ratio(Dim, 2),                       
    Semantic_shape == 'cylinder'. 

partAffordance(ID, pap:'AffordContain') :-
    primitive_values(ID, _, Dim, _, Semantic_shape),
    Semantic_shape == 'cylinder',  
    hard_condition_min(Dim, [0.03,0.03,0.03]),         
    max_shape(ID).

partAffordance(ID, pap:'AffordPound') :-
    primitive_values(ID, _, Dim, _, Semantic_shape), 
    min_dimension(Dim, [0.015, 0.015, 0.015]), 
    (Semantic_shape == 'cylinder' ;                     
        Semantic_shape == 'cuboid').

partAffordance(ID, pap:'AffordSupport') :-                            
    primitive_values(ID, _, Dim, _, _), 
    min_dimension(Dim, [0.0, 0.05, 0.05]),
    max_dimension(Dim, [0.03, 1, 1]),
    min_ratio(Dim, 3).

partAffordance(ID, pap:'AffordScoop') :-                 
    primitive_values(ID, _, Dim, _, _), 
    min_dimension(Dim, [0.01, 0.01, 0.01]),
    max_dimension(Dim, [0.1, 0.1, 0.1]),
    max_ratio(Dim, 2).                       

partAffordance(Affordance, Task) :- 
    disjoint_with(Task, Description),
    triple(Description, _, Affordance),
    subclass_of(Affordance, pap:'Affordance'). 

task_object(Affordance, Task) :- 
    subclass_of(Task, D), 
    triple(D, _, Affordance), 
    subclass_of(Affordance, pap:'Affordance'), 
    triple(D, _, pap:'RequiresAffordance').  
    
grasp_affordance(ID, Task) :-
    partAffordance(ID, Affordance), 
    subclass_of(Task, Description1), 
    triple(Description1, _, Affordance), 
    subclass_of(Affordance, pap:'Affordance'), 
    triple(Description1,_, pap:'hasGraspAffordance').