/*************************************************************************

    File: englishGrammar.pl
    Copyright (C) 2004,2005,2006 Patrick Blackburn & Johan Bos

    This file is part of BB1, version 1.3 (November 2006).

    BB1 is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    BB1 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BB1; if not, write to the Free Software Foundation, Inc., 
    59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*************************************************************************/

/*========================================================================
   Texts
========================================================================*/

%% Single Texts
t([sem:T])--> 
   s([coord:no,sem:S]),
   {combine(t:T,[s:S])}.

%% Complex Texts
t([sem:T])--> 
   s([coord:yes,sem:S]),
   {combine(t:T,[s:S])}.

%% Imperative Texts
t([sem:T])--> 
   s([coord:impera,sem:S]),
   {combine(t:T,[s:S])}.

t([sem:T])--> 
   q([sem:Q]),
   {combine(t:T,[q:Q])}.


/*========================================================================
   Sentences
========================================================================*/


%% Single Sentences
s([coord:no,sem:Sem])--> 
   np([coord:_,num:Num,gap:[],sem:NP]), 
   vp([coord:_,inf:fin,num:Num,gap:[],sem:VP]), 
   {combine(s:Sem,[np:NP,vp:VP])}.

%% Complex Sentences
%% Overlook
s([coord:yes,sem:_])--> [].

s([coord:yes,sem:Sem])--> 
   s([coord:ant,sem:S1]), 
   s([coord:con,sem:S2]), 
   {combine(s:Sem,[s:S1,s:S2])}.

s([coord:yes,sem:Sem])--> 
   s([coord:either,sem:S1]), 
   s([coord:or,sem:S2]), 
   {combine(s:Sem,[s:S1,s:S2])}.

s([coord:yes,sem:Sem])--> 
   s([coord:impera,sem:S1]), 
   s([coord:yes,sem:S2]), 
   {combine(s:Sem,[s:S1,s:S2])}.

%% Sub-Sentece
s([coord:ant,sem:Sem])--> 
   [if], 
   s([coord:no,sem:S]),
   {combine(s:Sem,[if:S])}.

s([coord:either,sem:Sem])--> 
   [either], 
   s([coord:no,sem:S]),
   {combine(s:Sem,[either:S])}.

s([coord:con,sem:Sem])--> 
   [then], 
   s([coord:no,sem:S]),
   {combine(s:Sem,[then:S])}.

s([coord:con,sem:Sem])-->
   s([coord:no,sem: S]),
   {combine(s:Sem,[then:S])}.

s([coord:or,sem:Sem])-->
   [or], 
   s([coord:no,sem:S]),
   {combine(s:Sem,[or:S])}.

sinv([gap:G,sem:S])-->
   av([inf:fin,num:Num,sem:Sem]),
   np([coord:_,num:Num,gap:[],sem:NP]),
   vp([coord:_,inf:inf,num:Num,gap:G,sem:VP]), 
   {combine(sinv:S,[av:Sem,np:NP,vp:VP])}.

%% Sentences From SCoTT

%% Imperative Sentences
s([coord:impera,sem:Sem])-->
   vp([coord:_,inf:inf,num:_,gap:[],sem:VP]), 
   {combine(s:Sem,[vp:VP])}.

%% Coupla + Predicative phase
s([coord:no,sem:Sem])--> 
   np([coord:_,num:Num,gap:[],sem:NP]), 
   cop([inf:_,num:Num,sem:_]),
   pred([sem:PRED]), 
   {combine(s:Sem,[np:NP,pred:PRED])}.

/*========================================================================
   Questions
========================================================================*/

q([sem:Sem])--> 
   whnp([num:Num,sem:NP]), 
   vp([coord:_,inf:fin,num:Num,gap:[],sem:VP]), 
   {combine(q:Sem,[whnp:NP,vp:VP])}.

q([sem:Sem])--> 
   whnp([num:_,sem:NP]), 
   sinv([gap:[np:NP],sem:S]),
   {combine(q:Sem,[sinv:S])}.


/*========================================================================
   Noun Phrases
========================================================================*/

np([coord:no,num:sg,gap:[np:NP],sem:NP])--> [].

np([coord:yes,num:pl,gap:[],sem:NP])--> 
   np([coord:no,num:sg,gap:[],sem:NP1]), 
   coord([type:conj,sem:C]), 
   np([coord:_,num:_,gap:[],sem:NP2]), 
   {combine(np:NP,[np:NP1,coord:C,np:NP2])}.

np([coord:yes,num:sg,gap:[],sem:NP])--> 
   np([coord:no,num:sg,gap:[],sem:NP1]), 
   coord([type:disj,sem:C]), 
   np([coord:_,num:sg,gap:[],sem:NP2]), 
   {combine(np:NP,[np:NP1,coord:C,np:NP2])}.

np([coord:no,num:sg,gap:[],sem:NP])--> 
   det([mood:decl,type:_,sem:Det]), 
   n([coord:_,sem:N]), 
   {combine(np:NP,[det:Det,n:N])}.

np([coord:no,num:sg,gap:[],sem:NP])--> 
   pn([sem:PN]), 
   {combine(np:NP,[pn:PN])}.

np([coord:no,num:sg,gap:[],sem:NP])--> 
   qnp([mood:decl,sem:QNP]), 
   {combine(np:NP,[qnp:QNP])}.


/*========================================================================
   WH Noun Phrases
========================================================================*/

whnp([num:sg,sem:NP])--> 
   qnp([mood:int,sem:QNP]), 
   {combine(whnp:NP,[qnp:QNP])}.

whnp([num:sg,sem:NP])--> 
   det([mood:int,type:_,sem:Det]), 
   n([coord:_,sem:N]), 
   {combine(whnp:NP,[det:Det,n:N])}.


/*========================================================================
   Nouns
========================================================================*/

n([coord:yes,sem:N])--> 
   n([coord:no,sem:N1]), 
   coord([type:_,sem:C]),  
   n([coord:_,sem:N2]),
   {combine(n:N,[n:N1,coord:C,n:N2])}.

n([coord:C,sem:Sem])--> 
   adj([sem:A,usage:attr]), 
   n([coord:C,sem:N]), 
   {combine(n:Sem,[adj:A,n:N])}.

n([coord:no,sem:Sem])--> 
   noun([sem:N]), 
   nmod([sem:PP]),
   {combine(n:Sem,[noun:N,nmod:PP])}. 

n([coord:no,sem:N])--> 
   noun([sem:Noun]),
   {combine(n:N,[noun:Noun])}.

nmod([sem:N])--> 
   pp([sem:PP,usage:attr]),
   {combine(nmod:N,[pp:PP])}.

nmod([sem:N])--> 
   rc([sem:RC]),
   {combine(nmod:N,[rc:RC])}.

nmod([sem:Sem])--> 
   pp([sem:PP,usage:attr]), 
   nmod([sem:NMod]),
   {combine(nmod:Sem,[pp:PP,nmod:NMod])}.


/*========================================================================
   Verb Phrases
========================================================================*/

vp([coord:yes,inf:Inf,num:Num,gap:[],sem:VP])--> 
   vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP1]), 
   coord([type:_,sem:C]), 
   vp([coord:_,inf:Inf,num:Num,gap:[],sem:VP2]),
   {combine(vp:VP,[vp:VP1,coord:C,vp:VP2])}.

vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
   av([inf:Inf,num:Num,sem:Mod]), 
   vp([coord:_,inf:inf,num:Num,gap:[],sem:V2]), 
   {combine(vp:VP,[av:Mod,vp:V2])}.

%% Copula + Predicative

vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
    cop([inf:Inf,num:Num,sem:Cop]), 
    np([coord:_,num:_,gap:[],sem:NP]), 
    {combine(vp:VP,[cop:Cop,np:NP])}.

%%  vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
%%     cop([inf:Inf,num:Num,sem:Cop]),    
%%     pred([sem:PRED]),
%%     {combine(vp:VP,[cop:Cop,pred:PRED])}.
%%     {combine(vp:VP,[pred:PRED])}.

%%  vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
%%     cop([inf:Inf,num:Num,sem:_]), 
%%     pp([coord:_,num:_,gap:[],sem:PP]), 
%%     {combine(vp:VP,[iv:PP])}.

vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
   iv([inf:Inf,num:Num,sem:IV]), 
   {combine(vp:VP,[iv:IV])}.

vp([coord:no,inf:I,num:Num,gap:G,sem:VP])-->   
   tv([inf:I,num:Num,sem:TV]), 
   np([coord:_,num:_,gap:G,sem:NP]), 
   {combine(vp:VP,[tv:TV,np:NP])}.

%% Verb Phrases From SCoTT
%% States Structure: Passive Voice

%% be iv-ed
vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
   cop([inf:Inf,num:Num,sem:_]),
   iv([inf:past,num:Num,sem:IV]), 
   {combine(vp:VP,[iv:IV])}, !.

%% be tv-ed
vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
   cop([inf:Inf,num:Num,sem:_]),
   tv([inf:past,num:Num,sem:TV]), 
   {combine(vp:VP,[iv:TV])}.

%% be Prep-ed
%% vp([coord:no,inf:Inf,num:Num,gap:[],sem:VP])--> 
%%  cop([inf:Inf,num:Num,sem:Cop]),
%%   pp([sem:PP]),
%%   {combine(vp:VP,[iv:TV])}.

/*========================================================================
   Prepositional Phrases
========================================================================*/

pp([sem:PP,usage:Usage])--> 
   prep([sem:Prep,usage:Usage]), 
   np([coord:_,num:_,gap:[],sem:NP]), 
   {combine(pp:PP,[prep:Prep,np:NP])}.

/*========================================================================
   Predicative Phrases
========================================================================*/

/* a box is on the shelf*/
pred([sem:PRED]) -->
   pp([sem:PP,usage:pred]),
   {combine(pred:PRED,[pp:PP])}.

/* a box is blue*/
pred([sem:PRED]) -->
   adj([sem:A,usage:pred]),
   {combine(pred:PRED,[adj:A])}.

/*========================================================================
   Relative Clauses
========================================================================*/

rc([sem:RC])--> 
   relpro([sem:RP]), 
   vp([coord:_,inf:fin,num:sg,gap:[],sem:VP]), 
   {combine(rc:RC,[relpro:RP,vp:VP])}.


/*========================================================================
   Lexical Rules
========================================================================*/

iv([inf:Inf,num:Num,sem:Sem])--> 
   {lexEntry(iv,[symbol:Sym,syntax:Word,inf:Inf,num:Num])},
   Word,
   {semLex(iv,[symbol:Sym,sem:Sem])}.

tv([inf:Inf,num:Num,sem:Sem])--> 
   {lexEntry(tv,[symbol:Sym,syntax:Word,inf:Inf,num:Num])},
   Word,
   {semLex(tv,[symbol:Sym,sem:Sem])}.

cop([inf:Inf,num:Num,sem:Sem])--> 
   {lexEntry(cop,[pol:Pol,syntax:Word,inf:Inf,num:Num])},
   Word,
   {semLex(cop,[pol:Pol,sem:Sem])}.

det([mood:M,type:Type,sem:Det])--> 
   {lexEntry(det,[syntax:Word,mood:M,type:Type])},
   Word,
   {semLex(det,[type:Type,sem:Det])}. 

pn([sem:Sem])--> 
   {lexEntry(pn,[symbol:Sym,syntax:Word])},
   Word,  
   {semLex(pn,[symbol:Sym,sem:Sem])}.

relpro([sem:Sem])--> 
   {lexEntry(relpro,[syntax:Word])},
   Word,
   {semLex(relpro,[sem:Sem])}.

prep([sem:Sem,usage:Usage])--> 
   {lexEntry(prep,[symbol:Sym,syntax:Word])},
   Word,
   {semLex(prep,[symbol:Sym,sem:Sem,usage:Usage])}.

adj([sem:Sem,usage:Usage])--> 
   % {lexEntry(adj,[symbol:Sym,syntax:Word])},
   {lexEntry(adj,[symbol:Sym,syntax:Word,catagory:Catagory])},
   Word,
   % {semLex(adj,[symbol:Sym,sem:Sem])}.
   {semLex(adj,[symbol:Sym,sem:Sem,catagory:Catagory,usage:Usage])}.

av([inf:Inf,num:Num,sem:Sem])--> 
   {lexEntry(av,[syntax:Word,inf:Inf,num:Num,pol:Pol])},
   Word,
   {semLex(av,[pol:Pol,sem:Sem])}.

coord([type:Type,sem:Sem])--> 
   {lexEntry(coord,[syntax:Word,type:Type])},
   Word, 
   {semLex(coord,[type:Type,sem:Sem])}.

qnp([mood:M,sem:NP])--> 
   {lexEntry(qnp,[symbol:Symbol,syntax:Word,mood:M,type:Type])},
   Word,
   {semLex(qnp,[type:Type,symbol:Symbol,sem:NP])}.

noun([sem:Sem])--> 
   {lexEntry(noun,[symbol:Sym,syntax:Word])},
   Word,
   {semLex(noun,[symbol:Sym,sem:Sem])}.

