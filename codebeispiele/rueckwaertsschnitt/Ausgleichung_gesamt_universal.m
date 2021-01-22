clc
clear all
format compact
format longg

%% Aufgabe: Bestimmung des Standpunkts mittels R�umlichen R�ckw�rtsschnitt
% anhand Messkonfiguration im Messkeller
% Skript auf tachymetrischen Messungen verwendbar.

%  Philip Brandst�tter & Finn Linzer
%  01527754
%  TU Wien
%  17.10.2019 &  23.06.2020
% 
%% Beobachtungen und Bekannte
%  Bekannte (Koordinaten der Festpunkte P21 - P73: 12 Punkte � 3
%  Koordinaten = 36 Koordinaten)
% 
%  Beobachtungen (Winkel zu Festpunkten: 12 Hz-Winkel, 12 V-Winkel) 

%% Einlese-Dateien
% Festpunktkoordinaten aus Lasertracker-Messungen
filename = 'LTD800_IA_160120.txt';     
delimiter = ',';
fp = importdata(filename,delimiter);

dataBeob = dlmread('IA_160120.txt');  % f�r Tachymeter-Messungen (Punktbezeichnung ohne Buchstaben
                                        % dlmread sorgt nur f�r numerische Dateneinlesung)
                                        
% Festpunktkoordinaten aus Laserscanner-Messungen
% filename = 'picking_list_reduced.txt';
% delimiter = ',';
% zp = importdata(filename,delimiter);

%% Festpunktkoordinaten (Beobachtungen) und Standpunktkoordinaten zuordnen
% Standpunktkoordinaten + gemessene Punkte
% wichtig! Korrekte Reihenfolge der Punkte 

% hochgenaue Tracker-Festpunktkoordinaten
fp_PN =  fp(:,1);               % Punktnummern der Festpunkte
fp = fp(:,2:4)/1000;            % /1000 um von mm- auf m-Werte zu kommen
fp_Y =   fp(:,1);               % Y-Koordinaten in Keller-System
fp_X =   fp(:,2);               % X-Koordinaten in Keller-System
fp_Z =   fp(:,3);               % Z-Koordinaten in Keller-System
fp_coord0 = [fp_Y fp_X fp_Z];

%% NUR F�R TACHYMETER-MESSUNGEN (!!!)
%  Die gemessenen Richtungen der tachymetrischen Messungen werden direkt
%  eingelesen und gelten anders als die berechneten Scanner-Beobachtungen 
%  als direkte Beobachtungen.

% Horizontalwinkel
L_Hz= dataBeob(:,2);
L_Hz_rad = L_Hz/200*pi;


% Vertikalwinkel
L_V = dataBeob(:,3);
L_V_rad = L_V/200*pi; 

%% Gewichtung
% Die Gewichtung der jeweiligen Messverfahren wird aus dem Datenblatt des
% Ger�ts entnommen oder individuell nach Fall angepasst.

% Umrechnungsvorschrift f�r Horizontalrichtungen: vom geod�tischen System ins mathematische System
L_Hz_rad = 2*pi - L_Hz_rad;


gon_o_r = 0.0005; % sind 3 [mgon] = 0.00004712 [rad]
rad_o_r = (gon_o_r/200)*pi;
o_r = rad_o_r.^2; 
ell_Hz = diag(o_r*ones(length(L_Hz),1));           
ell_V = diag(o_r*ones(length(L_V),1)); 
ell = blkdiag(ell_Hz,ell_V);
Qll = (1/o_r)*ell;
%% Iteration f�r Ausgleichsalgorithmus

% Abbruchbedingung f�r Iteration
EPSILON = 0.00000001;

% Unbekannte: 3D-Koordinaten des Standpunkts + Orientierungsunbekannte
% X = [ Y
%       X
%       Z
%       OU];

% N�herungskoordinaten
X0 = [   4
          50
          2
         0];   % Orientierungsunbekannte [radiant]

iteration = 1;
% NV = 4;
% while any(NV) > 3
while 1
    

% Berechnung der Horizontaldistanzen 
for i=1:length(L_Hz)
    s(i) = sqrt(((X0(1)-fp_Y(i))^2)+(X0(2)-fp_X(i))^2); 
end

% Berechnung der Schr�gdistanzen
for i=1:length(L_V)
    d(i) = sqrt(((X0(1)-fp_Y(i))^2)+((X0(2)-fp_X(i))^2)+((X0(3)-fp_Z(i))^2)); 
end

%% L0 - Vektor und verk�rzten Beobachtungsvektor definieren

% berechnete Richtungswinkel
%L0_Hz = riwi_rad(X0(1),X0(2),fp_coord0(:,1),fp_coord0(:,2))-X0(4);

pre_gon = atan2(   (fp_X)-X0(2)   , (fp_Y)-X0(1)   );
pre_gon = ((pre_gon*-1)/pi)*200;
pre_gon = pre_gon + ((X0(4)/pi)*200)

for k = 1:length(pre_gon)
    if pre_gon(k) < 0
    pre_gon(k) = pre_gon(k) + 400;
    end
end

L0_Hz = (pre_gon/200)*pi
L0_Hz = 2*pi - L0_Hz;


% verk�rzter Beobachtungsvektor der HZ
l_Hz = L_Hz_rad - L0_Hz;

    for i = 1:length(l_Hz)
       while (l_Hz(i,1)>  pi)
              l_Hz(i,1)=l_Hz(i,1)-2*pi;
       end
       while (l_Hz(i,1)< -pi)
              l_Hz(i,1)=l_Hz(i,1)+2*pi;
       end
    end

% berechnete Vertikalwinkel
for i=1:length(L_V)
%     L0_V(i) = acos((fp_Z(i)-X0(3))/d(i));       % auch �ber diese Formel m�glich
    L0_V(i) = atan2( s(i), fp_Z(i)-X0(3));
end


% verk�rzter Beobachtungsvektor der VZ
l_V = L_V_rad - L0_V';


L0 = [L0_Hz; L0_V'];
L = [L_Hz_rad; L_V_rad];

% gemeinsamer verk�rzter Beobachtungsvektor, S.124
l = [l_Hz; l_V]; 




%% Aufstellen der Designmatrix A
% A = partielle Ableitungen nach Unbekannten
% laut Niemeier S.151, Formel 4.4.12.
% siehe Ausgleichsrechnung Skriptum, S.126


% %%before 
% for i=1:length(L_Hz_rad)  
%     A_Hz(i,1) = fp_X(i)/(s(i)^2);           % Ableitung nach X
%     A_Hz(i,2) = -fp_Y(i)/(s(i)^2);          % Ableitung nach Y
%     A_Hz(i,3) = 0;                          % kein Einfluss der Hz-Richtungen auf H�henkomponente
%     A_Hz(i,4) = -1;                         % Ableitung nach Orientierungsunbekannte
% end
%%

for i=1:length(L_Hz_rad)  
    A_Hz(i,1) = (fp_X(i)-X0(2))/(s(i)^2);           % Ableitung nach X
    A_Hz(i,2) = -(fp_Y(i)-X0(1))/(s(i)^2);          % Ableitung nach Y
    A_Hz(i,3) = 0;                          % kein Einfluss der Hz-Richtungen auf H�henkomponente
    A_Hz(i,4) = -1;                         % Ableitung nach Orientierungsunbekannte
end

for i=1:length(L_V_rad) 
    A_V(i,1) = -((fp_Y(i)-X0(1))*(fp_Z(i)-X0(3)))/(s(i)*d(i)^2);  % Ableitung nach X
    A_V(i,2) = -((fp_X(i)-X0(2))*(fp_Z(i)-X0(3)))/(s(i)*d(i)^2);  % Ableitung nach Y
    A_V(i,3) = s(i)/d(i)^2;                                       % Ableitung nach Z
    A_V(i,4) = 0;                                                 % kein Einfluss auf Orientierungsunbekannte
end

%% auch mit dieser L�sung m�glich

% for i=1:length(L_V_rad) 
%     A_V(i,1) = -(cos(L0_V(i))*cos(L0_Hz(i)))/d(i);      % Ableitung nach X_N
%     A_V(i,2) = -(cos(L0_V(i))*sin(L0_Hz(i)))/d(i);      % Ableitung nach Y_N
%     A_V(i,3) = sin(L0_V(i))/d(i);                       % Ableitung nach Z_N
%     A_V(i,4) = 0;                                       % kein Einfluss auf Orientierungsunbekannte
% end

%% Ausgleichungsverfahren nach Gau�-Markov-Methode

Ages = [A_Hz; A_V];
A = Ages;

 P = inv(Qll);
 N = A'*P*A;  
 DetN = det(N);
 n = A'*P*l;      
 Qxx = inv(N);     
 disp('Werte der Ausgleichung')
 dx = Qxx*n;    
 X0(1) = X0(1) + dx(1);
 X0(2) = X0(2) + dx(2);
 X0(3) = X0(3) + dx(3); 
 X0(4) = X0(4) + dx(4); 
 X0
 dx

 iteration  = iteration + 1;

 betragdx = abs(dx);
    dxCheck = max(betragdx)
    if(dxCheck < EPSILON)
        break
    end
end

%% Umrechnungsvorschrift f�r tachymetrische Messungen von mathematischen in geod�tisches System
% bei negativer Orientierungsunbekannte wird 400Gon addiert, weil diese
% nicht negativ sein darf bzw. kann.
% 
% X0(4) = 100-(X0(4)*200/pi);
% if X0(4)<0
%     X0(4)=X0(4)+400;
% end

X0
%% Qualit�tsbeurteilung (nach Ausgleichsrechnung Skript, Niemeier und Neitzel)

% Proben von N und n
e = ones(1,4)';
s1 = A*e+l;
s2 = N*e +n;
if round(A'*P*s1,10) == round(s2,10)
    disp('Probe von N und n erfolgreich')
end
% Probe von Qxx
if round(e'*Qxx*N*e,10) == round(e'*e,10)
    disp('Probe von Qxx erfolgreich')
end

% Probe des Parametervektors dx
if round(e'*dx,10) == round(n'*(Qxx*e),10)
    disp('Probe von dx erfolgreich')
end

% Verbesserungsvektor + Probe
v = A*dx-l;
if round(A'*P*v,10) == 0
    disp('Probe von v erfolgreich')
end
% Anzahl der Freiheitsgrade
f = length(v)-length(dx);

I = eye(24);
%% Kofaktoren der Ausgleichung [bezogen auf die Varianz der Gewichtseinheit o0]
% Kofaktormatrix der ausgeglichenen Beobachtungen
Qldld = A*inv(N)*A';
% Kofaktormatrix der Verbesserungen 
Qvv = Qll -Qldld;
% Qvv = (Qvv.*rad_o_r)*200/pi;
% andere Berechnung f�r Qvv
Qv1v1 = Qll - Qldld;
% Test ob beide Berechnungen f�r Qvv dasselbe ergeben (--> Nullmatrix)
null = Qvv-Qv1v1;

ome = v'*P*v;
test1 = l'*P*l-n'*dx;
verb_gon = v*200/pi;

%% Globaltest
% empirische Standardabweichung der Gewichtseinheit in rad
s0_rad = sqrt(ome/f);
% empirische Standardabweichung der Gewichtseinheit in Gon
s0 = s0_rad*200/pi;
disp(s0);

% daraus kann man schlie�en: (Ausgleichsrechnung II Skriptum, S.20)
QXdXd = Qxx;
QLdLd = Qldld;
QLL = Qll;

%% Varianzen, Kovarianzen und Standardabweichungen der ausgeglichenen Gr��en [alle in Radiant]
% der ausgeglichenen Unbekannten
CXdXd = s0_rad^2*(diag(Qxx));
    % --> empirische Standardabweichungen der ausgeglichenen Unbekannten in
    % Radiant
    std_Unb = sqrt(CXdXd);
    std_Unb_OU = std_Unb(4)*200/pi; % in Gon
% der ausgeglichenen Beobachtungen
CLdLd = s0_rad^2*(diag(QLdLd));
    % --> empirische Standardabweichung der ausgeglichenen Beobachtungen in
    % Radiant
    std_Beob = sqrt(CLdLd);
    std_Beob_gon = sqrt(CLdLd)*200/pi; % in Gon
% der Verbesserungen in Radiant
Cvv = s0_rad^2*(diag(Qvv));
    % --> empirische Standardabweichung der ausgeglichenen Verbesserungen
    std_Verb = sqrt(Cvv);
    std_Verb_gon = sqrt(Cvv)*200/pi; % in Gon

% Probe der Gewichtsreziproke nach Ansermet:
% die Summe der Hauptdiagonalelemente der Produkmatrix P*QLL muss gleich
% der Anzahl der Unbekannten sein
if round(trace(P*Qll),1) == length(L) && round(trace(P*Qvv),1) == length(L)-length(dx)
    disp('Probe von Qvv erfolgreich')
end
nn = trace(P*Qll);
r = trace(P*Qvv);



%% 
% Differenz zwischen Varianzen der Verbesserungen und jenen der
% urspr�nglichen Beobachtungen wird als AUSGLEICHSGEWINN bezeichnet

% Redundanzanteile
% Berechnung der Redundanzanteile (gro�e Bedeutung bei Zuverl�ssigkeit)
rA = diag(P).*diag(Qvv);

% normierte Verbesserungen
sigv = diag(Qvv);
NV = abs(v)./(sqrt(o_r)*sqrt(sigv));
if any(NV > 3)
    disp('M�glichkeit eines groben Fehlers besteht');
end
if any(NV > 4)
    disp('Grober Fehler sehr wahrscheinlich');
end

% innere Zuverl�ssigkeit (???)
GF = -v./rA;
sigl = diag(Qll);
dNV = (GF.*sqrt(rA))./(sqrt(o_r)*sqrt(sigl));
d0 = 4.13;
pii = diag(P);
Grzw = (d0*sqrt(o_r))./(sqrt((rA).*pii)); % rad
Grzw = Grzw.*200/pi;

% �u�ere Zuverl�ssigkeit: Netzverzerrung
d0i = ((1-rA)./rA).*gon_o_r^2.*d0;

%% Durchf�hrung Globaltest
% H0: o0 = s0
% Beurteilung durch x�-Test
% Irrtumswahrscheinlichkeit a = 5%
a = 0.05;
% Anzahl Freiheitsgrade
f = f;
% Stichprobenfunktion 
o0 = gon_o_r;
X2dach = (f*(s0^2))/o0^2
% Wert aus Tabelle: 31.4 (f�r f = 20)
% wenn empirischer X2dach Wert gr��er ist als das Quantil der Verteilung,
% kann H0 verworfen werden
apos = s0/o0

%% Genauigkeitskriterien der Koordinatenunbekannten / Beobachtungen
% Standardabweichungen der Unbekannten
s_x = s0_rad*(sqrt(Qxx(1,1)))*1000; % in mm
s_y = s0_rad*(sqrt(Qxx(2,2)))*1000; % in mm
s_z = s0_rad*(sqrt(Qxx(3,3)))*1000;  % in mm
s_ou = s0_rad*(sqrt(Qxx(4,4)))*200/pi;  % in Gon


% Kovarianzmatrix der Unbekannten 
% 
% Exx = o0.^2*(Qxx);
% Exx_quer = s0_rad.^2*(Qxx);
% std_U = sqrt(diag(Exx_quer)); % muss gleich s_x,s_y,s_z und s_ou sein!!

% Standardabweichungen der Beobachtungen
% for i=1:length(L)
%     ql = diag(Qldld);
%     s_l(i) = s0_rad*sqrt(ql(i));
% end

% mmm = diag(QLdLd);

%% Helmert Punktlagefehler + Ellipsen

sH = sqrt(s_x^2 + s_y^2);
SH1 = s0^2*(Qxx(1,1)+Qxx(2,2));

www = (Qxx(1,1)-Qxx(2,2))^2 +4*Qxx(2,1)^2;
AF2 = 0.5*s0^2*(Qxx(1,1)+Qxx(2,2)+www);
BF2 = 0.5*s0^2*(Qxx(1,1)+Qxx(2,2)-www);
OK = atan((2*Qxx(2,1))/(Qxx(1,1)-Qxx(2,2)))*0.5*200/pi;

% Plot f�r Helmertsche Fehlerellipse

a=AF2/1000; % horizontal radius
b=BF2/1000; % vertical radius
x0=X0(1); % x0,y0 ellipse centre coordinates
y0=X0(2);
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y)

%% f�r Excel Export 

excel = [X0' s0 X2dach]'
sss = [s_x s_y s_z s_ou]'

%% Funktionen 

%/************************************************************************
%*  Funktion    : riwi_rad                         			             *
%*-----------------------------------------------------------------------*
%*  Aufgabe     : Berechnet Richtungswinkel vom Stand- zum Zielpunkt     *
%*  Parameter   : Koordinaten Zielpunkt, Koordinaten Standpunkt          *
%*  Return-Wert : Richtungswinkel im Bogenmass                           *
%************************************************************************/
function [t] = riwi_rad(x_stand,y_stand,x_ziel,y_ziel)
   
   t = atan2(y_ziel-y_stand,x_ziel-x_stand);
   
   for i=1:length(t)
       if t(i)<0
        t(i) = t(i)+2*pi;
       end
   end
end



