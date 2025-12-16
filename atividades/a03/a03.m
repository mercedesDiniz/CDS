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
    % Hz = [1 h1 h2 h3 h4];

    % Polinomio R(z) e S(z)
    r0 = 1;
    r1 = h4/da3;
    Rz = [r0 r1]; 

    s0 = ( h1-(r1+da1) )/b0b;
    s1 = ( h2-(da1*r1+da2) )/b0b;
    s2 = ( h3-(da2*r1+da3) )/b0b;
    Sz = [s0 s1 s2];

    % Polinomio T(z)
    toff = sum(Hcz)/sum(Bz); % pré-compensador DC
    Tz = toff*Hoz;
        t0 = Tz(1); t1 = Tz(2); t2 = Tz(3);  
    % Tz = [t0 t1 t2];

%% Resposta em frequência  
Cz = tf(Sz, Rz, Ts); % controlador
Gdlz = Cz*tf(b0b, DAz, Ts);       % sistema na malha direta

    % Análise de estabilidade relativa pelas curvas de sensibilidade
    Gclz = feedback(Gdlz, 1, -1);
    Tsen = Gclz;        % Func. de sensibilidade complmentar
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

%% Simulação no dominio do tempo
tfinal = 20;
N = round( tfinal/Ts );
t = 0:Ts:N*Ts-Ts;

    % Sinal de referencia
    % ref(1:N) = 0; ref(3:N) = 1; 
    ref(1:5)=0;
    ref(6: round( N/3 ))=1;
    ref(1+round( N/3 ): round(2*N/3) )=2.5;
    ref(1+round( 2*N/3 ): N )= 4;

    % Condições iniciais
    y(1:4) = 0; u(1:4) = 0; du(1:4) = 0;

for k = 5:N
    % Modelo da planta
    y(k) = -a1*y(k-1) -a2*y(k-2) +b0*u(k-1) +b1*u(k-2);

    % Controlador RST
    du(k) = -r1*du(k-1) +t0*ref(k) +t1*ref(k-1) +t2*ref(k-2) ...
        -s0*y(k) -s1*y(k-1) -s2*y(k-2);
    u(k) = u(k-1) +du(k);
end

% Plot
subplot(211)
    plot(t,ref,':k',t,y,'b');
    ylabel('Sinal de saída');
    legend('Ref.','y');
subplot(212)
    plot(t,u,'b');
    ylabel('Sinal de controle');
