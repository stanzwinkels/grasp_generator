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

% object_class(ID, pap:'Glass') :-
    % rules about the dimension of a glass
% object_class(ID, pap:'Hammer') :-
    % rule about the dimension of a hammer


% task_prim(ID, pap:'PourIn with Glass') :-
    % object_class('ID, pap:'Glass'), 
    % additional constraints

% task_prim(ID, pap:'HandOver with Bottle') :- 
% task_prim(ID, pap:'PourIn with Bottle') :- 
% task_prim(ID, pap:'PourOut with Bottle') :-
% task_prim(ID, pap:'Store in Shelf with Bottle') :-
% task_prim(ID, pap:'PlaceOn Table with Bottle') :-

% task_prim(ID, pap:'HandOver with Hammer') :- 
% task_prim(ID, pap:'Pound with Hammer') :- 
% task_prim(ID, pap:'Hang in Shelf with Hammer') :-
% task_prim(ID, pap:'PlaceOn Table with Hammer') :-

% task_prim(ID, pap:'HandOver with Ladle') :- 
% task_prim(ID, pap:'Scoop with Ladle') :- 
% task_prim(ID, pap:'Store in Shelf with Ladle') :-
% task_prim(ID, pap:'PlaceOn Table with Ladle') :-

% task_prim(ID, pap:'HandOver with Mug') :- 
% task_prim(ID, pap:'PourIn with Mug') :- 
% task_prim(ID, pap:'PourOut with Mug') :-
% task_prim(ID, pap:'Store in Shelf with Mug') :-
% task_prim(ID, pap:'PlaceOn Table with Mug') :-

% task_prim(ID, pap:'HandOver with Pan') :- 
% task_prim(ID, pap:'Cook with Pan') :- 
% task_prim(ID, pap:'PlaceOn Table with Pan') :-

% task_prim(ID, pap:'HandOver with Turner') :- 
% task_prim(ID, pap:'Support with Turner') :- 
% task_prim(ID, pap:'PlaceOn Table with Turner') :-