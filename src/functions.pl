% example of Prolog module defined by the user
% STATUS: working


:- module(functions,
    [
        float_decimal_accuracy/1,
        threshold/1,
        max_shape/1,
        min_shape/1,
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
        flatten/3, 
        roundD/3
        % primitive_values/4

    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

% :- dynamic primitive_values/4. 

% :- rdf_meta
%     primitive_values(+,+,+,+). 


max_shape(ID) :- 
    findall(V, 
        (primitive_values(_,_,Shape0,_), 
        volume3(Shape0, V)), 
            Elements),
    max_list(Elements, Max_value),
    roundD(Max_value, Max_Round), 
    primitive_values(ID,_,Shape,_),
    volume3(Shape, Vol), 
    roundD(Vol, Vol_round),
    Max_Round == Vol_round.

min_shape(ID) :- 
    findall(V, 
        (primitive_values(_,_,Shape0,_), 
        volume3(Shape0, V)), 
            Elements),
    min_list(Elements, Max_value),
    roundD(Max_value, Max_Round), 
    primitive_values(ID,_,Shape,_),
    volume3(Shape, Vol), 
    roundD(Vol, Vol_round),
    Max_Round == Vol_round.

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
    (2 - Threshold) < Eps1,
    Eps2 < Threshold,
    (Eps1 - (2- Threshold) - Eps2) >= 0.

shape_identification([Eps1,Eps2], cylindrical) :- 
    threshold(Threshold),
    (-Eps1 + Threshold - Eps2) < 0, 
    (Eps1 - (2 - Threshold) - Eps2) < 0,
    Eps2 < 1.6.

shape_identification([Eps1, Eps2], unknown) :- 
    \+ shape_identification([Eps1, Eps2], rectangular), 
    \+ shape_identification([Eps1, Eps2], cylindrical). 


