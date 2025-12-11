%% Atividade 3: 
% Conferir a estrutura RST em termos da sua sensibilidade aos ruídos de medição. 
% Verificar tanto a resposta em frequência (Tsen e Ssen), como a simulação com ruído Gaussiano nos sensores.

clear all; close all; clc;

%% Carregar um modelo previamente identificado para iniciar o controlador adaptativo.
load('modelo_Gz.mat');
Gz = modelo.Gz;
Ts = Gz.Ts;
    Az = Gz.den{1};
        a1 = Az(2); a2 = Az(3);
    Bz = Gz.num{1};
        b0 = Bz(2); b1 = Bz(3); b0b = b0+b1;

    % Ampliar o modelo para incluir a diferença discreta.
    % Δ = 1-z^-1
    DAz = conv(Az,[1 -1]);
        da1 = DAz(2); da2 = DAz(3); da3 = DAz(4);

%% Controlador RST 
Hoz = Az; % Polinomio de "observador": Ho(z) := A(z) -> quando os polos são estaveis

    wcl = 1;        % frequência [rad/s]
    zetacl = 1;     % fator de amortecimento
    Gcl = tf(wcl^2,[1   2*zetacl*wcl  wcl^2]);
    Gzcl = c2d(Gcl,Ts);

Hcz = Gzcl.den{1}; % Polinomio desejado em M.F: Hc(z)

Hz = conv(Hcz,Hoz);
    h1 = Hz(2); h2 = Hz(3); h3 = Hz(4); h4 = Hz(5);

    % Polinomio R(z) e S(z)
    r1 = h4/da3;
    Rz = [r1 0]; 

    s0 = ( h1-(r1+da1) )/b0b;
    s1 = ( h2-(da1*r1+da2) )/b0b;
    s2 = ( h3-(da2*r1+da3) )/b0b;
    Sz = [s0 s1 s2];

    % Polinomio T(z)
    toff = sum(Hcz)/sum(Bz); % pré-compensador DC
    Tz = toff*Hoz;
        t0 = Tz(1); t1 = Tz(2); t2 = Tz(3);  

%% Resposta em frequência
Cz = tf(Rz, Sz, Ts);    % controlador
Gdlz = Cz*Gz;           % sistema na malha direta

    % Análise de estabilidade relativa pelas curvas de sensibilidade
    Gclz = feedback(Gdlz, 1, -1);
    Tsen = Gclz;        % Func. de sensibilidade complmentar = sistema em malha fechada
    Ssen = 1 - Tsen;    % Func. de sensibilidade

    figure; sigma(Tsen); hold; sigma(Ssen); grid;
    % title('Funções de sensibilidade');
    legend('|T(e^{j\omegaT_s})|','|S(e^{j\omegaT_s})|');
    
    % Picos de sensibilidade
    mt = max( max( sigma(Tsen) ) );
    ms = max( max( sigma(Ssen) ) );
    
    % Margens de ganho e fase
    GmdB = min( 20*log10(ms/(ms-1)), 20*log10(1+(1/mt)) )
    Pmdeg = (180/pi)*min( (2*asin(1/(2*ms)) ), (2*asin(1/(2*mt)) ) )