clear all
close all

k = 10*10^-12;
E0 = 1.259;

% Species ordering is c1 c2 c1 c0
D = [-k 0 0 0; 0 0 0 0; 2*k 0 0 0; -k 0 0 0];

currentsigns = [1;-1;-1;1];

V_tank = 0.05; V_cell = 9.5*10^-4; N = 19; F = 96845; R = 8.314; S = 0.15; d = 1.27*10^-4; z = 1;

c_t = [0;2500;2500;0];
c_c = [0;2500;2500;0];

NumSteps = 24*3600;
Q = 100*1.66*10^-5*[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 0;];
I = 150;
cntr = 0;
holup = 0;
waittime = 1200;

for ii = 1:NumSteps
    SOC(ii) = c_t(1,ii)/(c_t(1,ii)+c_t(2,ii));
    if 1-SOC(ii) <= 0
        compfac = 0;
    else
        compfac = 1;
    end
    d_cell = c_t(:,ii)-c_c(:,ii);
    c_c(:,ii+1) = c_c(:,ii)+1/V_cell*(Q*d_cell+1/(z*F)*currentsigns*I(ii)+S/d*D*compfac*c_c(:,ii)); 
    c_t(:,ii+1) = c_t(:,ii)+1/V_tank*(N*Q*(c_c(:,ii+1)-c_t(:,ii)));
    if holup ~= 0
        I(ii+1) = 0;
        cntr = cntr + 1;
        if cntr > waittime
            I(ii+1) = sign(holup)*150;
        end
        if cntr > waittime+120
            holup = 0;
            cntr = 0;
        end
    else
        if (SOC(ii) > 0.99)
            holup = -1;
            I(ii+1) = 0;
        elseif (SOC(ii) < 0.01)
            holup = 1;
            I(ii+1) = 0;
        else
            I(ii+1) = I(ii);
        end
    end
    V(ii) = N*(E0+R*293.15/F*log((c_t(1,ii+1)/c_t(2,ii+1)+0.1)/(c_t(4,ii+1)/c_t(3,ii+1)+0.1)));
end

%%

figure(1)
hold on
for ii = 1:size(c_c,1)
    plot(c_c(ii,:))
end
legend('$c_{2a}$','$c_{1a}$','$c_{1c}$','$c_{0c}$')
hold off
figure(2)
hold on
for ii = 1:size(c_t,1)
    plot(c_t(ii,:))
end
hold off
legend('$c_{2a}$','$c_{1a}$','$c_{1c}$','$c_{0c}$')

figure(4)
subplot(3,1,1)
plot(V)
legend('Stack voltage')
subplot(3,1,2)
plot(I)
legend('Stack current')
subplot(3,1,3)
plot(SOC)
legend('Anolyte-side SOC')
