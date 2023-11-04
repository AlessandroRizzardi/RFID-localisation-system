% PARAMETRI ROBOT
dVera = 26; % distanza vera tra le due ruote del robot
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
KRvera = 0.01;   % Costante KR errori odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori odometrici ruota sinistra: uL_e = N(uL, KL |uL|)

% ERRORI SISTEMATICI
d = dVera;%*1.001; % distanza presunta tra le due ruote, se d = dVera non c'� errore sistematico
deltaR = deltaRvera;%*1.001; % conoscenza (eventualmente approssimata) di deltaRvera
deltaL = deltaLvera;%*1.001; % conoscenza (eventualmente approssimata) di deltaLvera
KR = KRvera;%*1.01; % conoscenza (eventualmente approssimata) di KR 
KL = KLvera;%*1.01; % conoscenza (eventualmente approssimata) di KL

passoOdometrico = 1; % costante per generazione cammino

% CAMMINO
nPassi = 1000; % numero passi cammino

% Definizione dei vettori x,y e theta delle coordinate del robot durante la simulazione:
xVett = zeros(nPassi,1);
yVett = zeros(nPassi,1);
thetaVett = zeros(nPassi,1);

clearance = 75; % distanza dal bordo per cui si cambia direzione

deltaTheta = zeros(nPassi,1);

angoloRand = 2*pi*rand;
xVett(1) = 100+clearance*cos(angoloRand);%clearance;% + rand*(L-2*clearance);
yVett(1) = 100+clearance*sin(angoloRand);%70;% clearance + rand*(L-2*clearance);
thetaVett(1)=0;%360*rand*gradi; % angolo in radianti: gradi e' il fattore di conversione
deltaRho = passoOdometrico*ones(nPassi,1);

uR = zeros(nPassi,1);
uL = zeros(nPassi,1);

angoloMaxCurva = 100; % angolo max di curva totale
angoloMaxStep = 5; % angolo max di curva in ogni step

k = 1;
while k < nPassi

    [lato, distanza] = distanzaBordo(xVett(k),yVett(k),thetaVett(k));
    distanzaTag = Inf;
    if distanzaTag < 20
        if abs(atan2(cTag(1,2)-yVett(k),cTag(1,1)-xVett(k))-thetaVett(k)) > 20*pi/180
            distanzaTag = Inf;
        end
    end

    if distanza < clearance || distanzaTag < 20 % devo curvare
        curvaDaFare = max(angoloMaxStep,rand*angoloMaxCurva); % � la curva in gradi che deve essere effettuata
        passiCurvaDaFare = round(curvaDaFare/angoloMaxStep); % � il numero di passi che il robot impiegher� per fare la curva
        gradiPerPasso = curvaDaFare/passiCurvaDaFare;
        kIn = min(k,nPassi);
        kFin = min(k+passiCurvaDaFare,nPassi-1);
        indiceK = [kIn:kFin];
        direzione = mod(round(thetaVett(k)/gradi),360); % direzione in gradi tra 1 e 360        
        deltaRho(indiceK)=0; % mi fermo
        if distanzaTag < 20 % mi sto dirigendo sotto il tag
            deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
        end
        if lato == 1 % mi sto dirigendo contro il lato sotto
            if direzione>270 % sto puntando verso destra
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
            else
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso sinistra perch� sto puntando verso sinistra
            end
        elseif lato == 2 % mi sto dirigendo contro il lato destro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso il basso
            else
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso l'alto perch� sto puntando verso l'alto
            end
        elseif lato == 3 % mi sto dirigendo contro il lato sopra
            if direzione<90 % sto puntando verso destra
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso destra
            else
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso sinistra perch� sto puntando verso sinistra
            end            
        elseif lato == 4 % mi sto dirigendo contro il lato sinistro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso il basso
            else
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso l'alto perch� sto puntando verso l'alto
            end
        end 
        for k = kIn:kFin
            uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
            uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

            xVett(k+1)= xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
            yVett(k+1)= yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
            thetaVett(k+1)= thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
%                 k
        end   
    else
        uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
        uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

        xVett(k+1)= xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
        yVett(k+1)= yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
        thetaVett(k+1)= thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
    end
    
    k = k + 1;        
    
end

% Passaggio dagli spostamenti deltaRho e deltaTheta agli spostamenti delle
% due ruote
uR = (deltaRho + (dVera/2)*deltaTheta)/deltaRvera;
uL = (deltaRho - (dVera/2)*deltaTheta)/deltaLvera;

x0 = xVett(1);
y0 = yVett(1);    
theta0 = thetaVett(1);    

% Errori odometrici
NiR = randn(nPassi,1)*sqrt(KRvera); 
NiL = randn(nPassi,1)*sqrt(KLvera); 

% Letture odometriche corrotte
uRe = zeros(nPassi,1); % vettore letture odometriche ruota destra
uLe = zeros(nPassi,1); % vettore letture odometriche ruota sinistra
for k = 1:nPassi
    uRe(k) = uR(k) + sqrt(abs(uR(k)))*NiR(k); %encoders reading - letture odometriche ruota destra
    uLe(k) = uL(k) + sqrt(abs(uL(k)))*NiL(k); %encoders reading - letture odometriche ruota sinistra
end

% Disturbo letture fase
NNfase = randn(nTag,nPassi); 