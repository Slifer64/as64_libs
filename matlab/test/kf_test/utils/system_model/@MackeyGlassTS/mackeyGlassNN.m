function [Y,Xf,Af] = mackeyGlassNN(X)
%MACKEYGLASSNN neural network simulation function.
%
% Generated by Neural Network Toolbox function genFunction, 06-Jul-2019 12:44:51.
% 
% [Y,Xf,Af] = mackeyGlassNN(X,Xi,~) takes these arguments:
% 
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = 1xQ matrix, input #1 at timestep ts.
% 
%   Xi = 1x7 cell 1, initial 7 input delay states.
%   Each Xi{1,ts} = 1xQ matrix, initial states for input #1.
% 
%   Ai = 3x0 cell 3, initial 7 layer delay states.
%   Each Ai{1,ts} = 20xQ matrix, initial states for layer #1.
%   Each Ai{2,ts} = 10xQ matrix, initial states for layer #2.
%   Each Ai{3,ts} = 1xQ matrix, initial states for layer #3.
% 
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = 1xQ matrix, output #1 at timestep ts.
% 
%   Xf = 1x7 cell 1, final 7 input delay states.
%   Each Xf{1,ts} = 1xQ matrix, final states for input #1.
% 
%   Af = 3x0 cell 3, final 0 layer delay states.
%   Each Af{1ts} = 20xQ matrix, final states for layer #1.
%   Each Af{2ts} = 10xQ matrix, final states for layer #2.
%   Each Af{3ts} = 1xQ matrix, final states for layer #3.
% 
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = 0.37596451683883;
x1_step1.gain = 1.40893052952284;
x1_step1.ymin = -1;

% Layer 1
b1 = [2.1603978173360172121;2.0085599246449055144;1.3274652204298229563;1.5828584637513716249;-1.1498740277513830321;0.74143132300242520216;1.3710713839107113721;0.29501255171710633052;-0.61876822959793720269;-0.23654547769849801342;0.27139563791960069628;-0.44578938746323532039;0.51487942237379902277;-0.98323532457931273054;-1.6609948573234332159;-1.0565823152063646972;-1.7238607730078838731;-1.7715802591651712561;-2.3481638016380119716;-2.2573417883343092072];
IW1_1 = [-1.1942484424745787308 -0.51352704504663149176 0.66836502443197098877 -0.031091278138664107944 0.9395496459343151141 0.45283038357554872277 -0.38162277276058403919;-0.41555679311780824303 -0.20601003386999849898 0.14751950752982601056 -1.1422214393008967015 -0.22188811798628144123 1.2982795446452326527 0.98772854752094185748;-1.1290680041583593951 -0.58819951349181964773 0.51894207423611093954 0.31651459805807252712 0.77260452828365044908 0.12367844592946862503 -0.84340808616269424824;-0.89577446897875889764 -0.71643933093455580696 -0.77863985708858418633 0.45819537252038500252 0.71942391841185227008 -0.95424716774719153811 -0.77991463778382252858;0.64210049172109440185 -0.85160552446158177542 0.37759240028878232609 -0.55104552370078851986 0.16508479866956193849 1.1942826316405077414 -1.2033949198097508848;-1.0691317683584324882 -0.25333252829270008766 -0.49169256783247294873 0.50489007083248060859 -0.27677875547481239904 -0.0006909010844744641576 -0.52926454881749485093;-0.16876381474051863063 0.45508170934944502939 0.68577013569202482923 1.1679963519604732447 -0.30315346517037855545 -0.51029261690536176221 0.92720119357219155098;-0.31039655743635186091 0.74973469140499593433 -0.34396855435892148556 0.7860268390321073495 -0.24220068941993277978 1.4894700168830614651 -0.89528810052112939299;-0.46809831651145666909 0.46098023508330737696 -0.99114871196480069937 0.97786854423510027878 -0.56651520019635193126 0.9897821021681808551 -0.42737146655195218381;0.78074405574681748909 0.68137747032181972262 -0.6303815500960405771 -0.20579118925020001285 -0.81500210463403111039 0.37281187286993150432 -0.81370036788875110823;0.21369623790536299013 -0.24899875122153178331 -0.0068948700611322910614 0.83480703792882848813 0.47270448050809033003 -0.1238759876870031279 -1.0645379500251561655;-0.34837318430141933634 -0.26394815964041495882 0.016011118145415413616 0.61057871880576808366 -0.354155598200032673 -1.8284140520860958645 -0.26852095899060102191;-0.2253295118155352883 -1.0778941477581023278 0.69112957251626594424 1.0446460988212911136 -0.761018463291486702 -0.32302619621462830279 0.67936061260179392285;-1.255195969778250431 -1.0114180463394502052 0.24112691975368402697 0.7368476327672519588 0.071695863933381509536 -0.65105744984632074601 -0.55921671866636013526;-1.0736286903386160763 0.30069067404585692049 -0.89012302786376196195 0.37011387451717681074 0.78579308693489946513 -0.0085404810329647619371 0.55640345493210974226;-1.4752160813634431769 0.34876013927454097008 -0.029808599191066525308 -0.52693898777526626542 -0.38326503955589152906 -0.47595753011328195692 0.76081118339990672084;-0.8666549542244768034 0.072866383275610277503 -0.98337391263386364493 -0.20952531626615750193 -0.034044679088583551518 -1.3224290903108699702 0.067773483417016042174;-1.1026768955523134075 -0.065732726533529142743 0.96000083951328030185 0.37732312702342091226 0.23604195379299572344 -0.83017529019778224519 1.269684382775550624;-0.59868102912374954183 -0.069832830408380466847 -0.75271446471160075031 0.84571663265371699314 0.15216552579617464991 1.0372238007274046101 -1.07916506787631028;-0.26879180124854662459 1.0769793085990548853 -1.2175640042450435185 0.18659801324439970194 -0.012846859964632372647 -0.99528186311157906818 0.7055470012880603603];

% Layer 2
b2 = [-1.6431596053120987033;-1.2038565563128937885;0.86934055855980296723;0.43466276743505588565;0.22352803204574539508;-0.25509934200038636432;-0.39763656334532154757;-0.90220530796754205838;1.391215062668408553;1.7161329299952312599];
LW2_1 = [0.17604537250484628141 0.08560655740623764054 0.26876700599280806125 0.30104058628342850179 0.17784395937710290037 0.15575469811100836615 -0.22368185804820631124 0.56335955749113741931 0.11003686849559703165 -0.34086782073615395916 -0.26635402861580614875 -0.55149640512483200094 0.20435286440897432181 0.25180550835793530373 0.26744268370956963965 -0.45500038741532738973 0.4307864262825774615 -0.35715275618477165809 0.011654125301544469503 0.092855379475436441994;0.447032053558072906 0.29215280004701515981 0.72543413835970926851 -0.032116006539870380065 0.28550206108669567984 -0.058994402319779731814 -0.063577745037610253198 0.81075993050434391751 0.39539964768975882636 -0.071428488269432044433 -0.49615221372675505362 -0.94302625644672566363 0.016879664868221511131 0.3620402764501820525 0.29900182651644807708 -0.12326009251360579844 -0.083597336488063930804 -0.11796382373093219909 -0.32550280721952923324 -0.12067116457153652387;-0.31585246325973886927 0.35802369491179492078 0.55968691185406949096 -0.59208248028208720903 -0.43928865054300764159 0.40171237745139348752 -0.019325656677396146205 0.13745978214695958597 0.0062267062802933200588 0.29142477681356387942 -0.3415728996716466126 -0.074681902996000623651 0.49453702652866260436 -0.20061950895106167581 -0.49719707302929561088 0.027288429091444948299 0.18820534992078716274 -0.54961065830300004365 -0.17226532718147752332 -0.30469604449827597525;0.0020611031234998170372 0.52236040071594269651 -0.27445415424951524752 0.069552545435722826217 0.49177786315619392843 -0.3930379606848486973 0.080394928413520541643 -0.081038860694460812528 0.35664417559217453935 -0.48326209089428279198 0.0093599664954116760379 -0.32001924885849208913 0.36747062362417381998 -0.38467791500881454159 -0.65160120193491333218 -0.348965934331879446 -0.10047024649742600233 -0.24699796234691073771 0.24866045779020382045 0.0056727204305126403133;-0.55562316183131588154 -0.13906450330609476662 0.16545935285265070402 -0.61320819192654285601 -0.41867985199189189371 0.30437587022619821964 -0.11409273864585343927 0.079151167682882897503 -0.49349369885897736987 0.0028143216199487883222 0.39574959929371356093 0.12487742034920802647 0.27471927645228166881 -0.3436779581259983285 -0.18034281946003746788 0.86873005179197893 -0.48084291880059498725 0.1777794474439652328 0.35935244929298831851 0.44640450237566764446;-0.34139977211566890913 0.23553693627975999525 -0.2057008963845587679 -0.1350909043965219658 -0.02823241782664208388 -0.041767400264895684914 -0.77416499158615503795 -0.62688812374374447423 0.15364119679706475519 0.37204283633201185033 -0.37521527541930776994 -0.2725296672003872378 0.16534143844127324741 0.6200035755477490973 -0.54888886292068539419 -0.2930428499347123017 0.43434899015573674719 0.44106216357995337018 -0.058513680028556017132 -0.16514643096975673009;-0.28090450072100259238 0.17855129298038988073 -0.042399941430040173629 0.2185542946501957684 0.46367193503635595331 0.61336863170096300735 -0.082067955492640912163 -0.098009826675172023736 -0.11925695926379215051 -0.3526399063194691208 -0.42184166744771506297 0.33678568260399566192 -0.040105121530375348327 0.4151868342180331517 -0.14237713153462214222 0.045514425631762576996 0.28989731150620012823 0.012587542346960783196 -0.29562804311209395225 -0.48654376430424661759;-0.52596221606118365166 -0.64809135977773213444 -0.081097597461202139413 -0.15783615276628856772 0.51381429716608451042 -0.21808369277904424144 0.33208522151286118396 -0.18087884612600613199 0.70478428268997850292 0.11691092540989686532 -0.48978583636400613299 0.40032935013937515079 -0.38632295486161166709 0.10641875092041667983 -0.21347933115147049721 -0.023947216025960341851 -0.26759027920044381466 0.11768742601261293623 0.084431763653545738313 0.31668307054831584813;0.0016236673058855925419 0.49862228298558036732 0.48217897422335526114 0.40631634725084991855 -0.21634272821870054182 0.096654779082747360741 0.037149977524880507807 -0.37687059595877514528 0.3547434126287802858 -0.17167566593405725328 -0.78822582251683459553 0.21335137703555756561 -0.52430496261294634586 0.058656373200923976607 0.53838764691957430308 -0.023007641687789424922 0.28344855156271941965 0.43241442812254132777 -0.49884642977630788696 0.1759661705114873298;0.38354363583931128101 -0.3525305229040557875 0.26562094732252233831 -0.40869860866586144033 -0.44437723166913639661 -0.21801076882721140415 0.25164136101822154323 -0.27269440238380432406 -0.21455116018994321858 -0.4412090255296755581 -0.16103088735230436379 0.12075847375440505738 0.4634868097455882574 -0.42285881422844695532 0.48003740182322085506 -0.16371753216849671553 0.3187557208431046063 -0.26740704975056639459 0.46536202734856052521 -0.23374808885513551071];

% Layer 3
b3 = 0.59823166621445589275;
LW3_2 = [-0.38898593743238396581 -0.53359860300659800458 -0.38137411092716383409 0.51922405365038681868 -0.4427311726565196226 0.048930443279848516436 -0.29578449143206342331 0.11538615133200637308 -0.47312284624086264895 -0.6821886641030717735];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 1.40893052952284;
y1_step1.xoffset = 0.37596451683883;

% ===== SIMULATION ========

X = num2cell(X);
Xi = X;

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX, X = {X}; end;
if (nargin < 1), error('Initial input states Xi argument needed.'); end

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
  Q = size(X{1},2); % samples/series
elseif ~isempty(Xi)
  Q = size(Xi{1},2);
else
  Q = 0;
end

% Input 1 Delay States
Xd1 = cell(1,8);
for ts=1:7
    Xd1{ts} = mapminmax_apply(Xi{1,ts},x1_step1);
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS

      % Rotating delay state position
      xdts = mod(ts+6,8)+1;
    
    % Input 1
    Xd1{xdts} = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    tapdelay1 = cat(1,Xd1{mod(xdts-[1 2 3 4 5 6 7]-1,8)+1});
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*tapdelay1);
    
    % Layer 2
    a2 = tansig_apply(repmat(b2,1,Q) + LW2_1*a1);
    
    % Layer 3
    a3 = repmat(b3,1,Q) + LW3_2*a2;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a3,y1_step1);
end

% Final Delay States
finalxts = TS+(1: 7);
xits = finalxts(finalxts<=7);
xts = finalxts(finalxts>7)-7;
Xf = [Xi(:,xits) X(:,xts)];
Af = cell(3,0);

% Format Output Arguments
if ~isCellX, Y = cell2mat(Y); end

Y = cell2mat(Y);

end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
  y = bsxfun(@minus,x,settings.xoffset);
  y = bsxfun(@times,y,settings.gain);
  y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
  a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
  x = bsxfun(@minus,y,settings.ymin);
  x = bsxfun(@rdivide,x,settings.gain);
  x = bsxfun(@plus,x,settings.xoffset);
end