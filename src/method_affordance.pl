% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(method_affordance,
    [
        float_decimal_accuracy/1,
        threshold/1,
        soft_condition_min/2,
        soft_condition_max/2,
        hard_condition_min/2,
        hard_condition_max/2,
        max_shape_difference/2,
        min_shape_difference/2, 
        flatten2D/2,
        roundD/2,   
        roundlist/2,
        volume3/2,
        max_volume/2,
        shape_identification/2,
        shape/2,
        find_categorical_shapes/2,  
        categorical_matching/2, 
        categorical_matching/2, 
        task_object/2,
        task_component/2,
        object_component/2,
        flatten/3, 
        roundD/3,
        select_primitive/4,
        primitive_values/4

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
    
% soft constraint for minimum
soft_condition_min([X,Y,Z], T) :-
    findall(Element, 
            (nth0(_, [X,Y,Z], Element), X > T; Y > T; Z > T),
            Elements),
    Elements \== [].

% soft constraint for maximum
soft_condition_max([X,Y,Z], T) :-
    findall(Element, 
            (nth0(_, [X,Y,Z], Element), X < T; Y < T; Z < T),
            Elements),
    Elements \== [].

% hard constraint for minimum
hard_condition_min([X,Y,Z], [Tx, Ty, Tz]) :- 
    findall(Element, (nth0(_, [X,Y,Z], Element), X > Tx, Y > Ty, Z > Tz), Elements), 
    Elements \== [],
    Elements == [X,Y,Z].

% hard constraint for maximum
hard_condition_max([X,Y,Z], [Tx, Ty, Tz]) :- 
    findall(Element, (nth0(_, [X,Y,Z], Element), X < Tx, Y < Ty, Z < Tz), Elements), 
    Elements \== [],
    Elements == [X,Y,Z].

max_shape_difference(Threshold, List) :- 
    forall(member(A, List), 
        (forall(member(B, List), 
            (
                Low is B/Threshold,
                Up is B*Threshold, 
                Low < A, 
                A < Up
                )))).

min_shape_difference([A,B,C], Threshold) :- 
    (Threshold*B < A, 
    Threshold*C < A) ;
    (Threshold*C < B, 
    Threshold*A < B) ;
    (Threshold*A < C, 
    Threshold*B < C). 


% Flatten a combined list [[x,x][x,x]] --> [x,x,x,x]
flatten( [], Z, Z):- !.                                       
flatten( [Atom|ListTail], [Atom|X], Z) :-                      
    \+is_list(Atom), !,                                       
    flatten( ListTail, X, Z).                                
flatten( [List|ListTail], X, Z) :-                            
    flatten( List,     X, Y),      
    flatten( ListTail, Y, Z).       
flatten2D(A, B) :- flatten( A, B, []).


% Rounding of numbers to a certain decimal. 
float_decimal_accuracy(6).  
roundD(X, Y):-
	float_decimal_accuracy(D),
	roundD(X, Y, D).

roundD('unknown', 'unknown', _):- !.                                % value unknown
roundD(X, Y, D) :- Z is X * 10^D, round(Z, ZA), Y is ZA / 10^D.     % round to decimal X

roundlist(In, Out) :- 
    findall(Out, 
        ((nth0(_, In, Element)), 
        roundD(Element, Out)), Out).


volume3([A,B,C], Volume) :- 
    Volume is (A * B * C).

max_volume(Lists, ID) :- 
    findall(Volume, 
        (nth0(_, Lists, List),
        roundlist(List, RList),  
        volume3(RList, Volume)), 
        Volume), 
    max_list(Volume, Max_volume),
    primitive_values(ID,_,Shape,_),
    roundlist(Shape, RShape),
    volume3(RShape, Volume), 
    Volume == Max_volume. 


threshold(0.7).

shape_identification([Eps1,Eps2], rectangular) :- 
    threshold(Threshold),
    Eps1 < Threshold,
    Eps2 < Threshold, 
    (-Eps1 + Threshold - Eps2) >= 0.

shape_identification([Eps1,Eps2], rectangular) :- 
    threshold(Threshold),
    Eps1 < Threshold,
    Eps2 > (2 - Threshold),
    (Eps1 - (2- Threshold) - Eps2) >= 0.

shape_identification([Eps1,Eps2], cylindrical) :- 
    threshold(Threshold),
    (-Eps1 + Threshold - Eps2) < 0, 
    (Eps1 - (2 - Threshold) - Eps2) < 0,
    Eps2 < 1.6.

shape_identification([Eps1, Eps2], unknown) :- 
    \+ shape_identification([Eps1, Eps2], rectangular), 
    \+ shape_identification([Eps1, Eps2], cylindrical). 


% return all ID that match the condition
shape(ID, pap:'Cap') :-                                           % rules for a cap  
    primitive_values(ID, EPS, Shape, _), 
    hard_condition_max(Shape, [0.04,0.04,0.04]),            % x,y,z smaller than 4 cm 
    hard_condition_min(Shape, [0.01,0.01,0.01]),            % x,y,z greater than 1 cm
    max_shape_difference(2.2, Shape),                       % compare shape difference x, y, z with threshold (1.3)
    (shape_identification(EPS, cylindrical) ; 
        shape_identification(EPS, unknown)).

shape(ID_final, pap:'Body') :-                                    % rules for a body
    findall(Shape,    
        (primitive_values(_, _, Shape, _), 
        hard_condition_min(Shape, [0.01,0.01,0.01]),         % x,y,z greater than 4 cm
        hard_condition_max(Shape, [0.15,0.15,0.15]),         % x,y,z smaller than 15 cm (small products)    
        soft_condition_min(Shape, 0.04),
        max_shape_difference(5, Shape)
     ), Shape), 

    % find biggest primitive
    flatten2D(Shape, Shape_flat),                     % flatten the list
    roundlist(Shape_flat, Shape_round),             % roundoff to x digits
    % max_volume(Shape_round, ID_final).
    max_list(Shape_round, Max_value),               % find max value
    primitive_values(ID_final, EPS, Shape1, _),       % iterate through primitives
    roundlist(Shape1, Shape_round1),                % roundoff to x digits
    member(Max_value, Shape_round1),                % find max value in primitive_values
    (shape_identification(EPS, cylindrical);
        shape_identification(EPS, rectangular)). 

shape(ID, pap:'Handle') :-                                    % rules for a handle
    primitive_values(ID, EPS, Shape, _),
    hard_condition_min(Shape, [0.01, 0.01, 0.01]),
    hard_condition_max(Shape, [0.1, 0.1, 0.1]),
    soft_condition_min(Shape, 0.025),
    soft_condition_max(Shape, 0.025),                   % atleast one shape is between certain parameters
    min_shape_difference(Shape, 2),                     % atleast one axis is x times as long as the others
    shape_identification(EPS, cylindrical).             % handle must be of shap rectangular


% Assigns categories to shape primitives
find_categorical_shapes(ID, List) :- 
    primitive_values(ID, _, _, _),      % check all shape primitives
    findall(List, shape(ID, List), List).  % categorizes shape primitives
        
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


