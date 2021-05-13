clear, clc
%
%параметры модели модели (взяты из источника)
long=0.94;%длина модели
heigh=0.2;%высота модели
weigh=0.63;
lf=long/2;%длина от середины до передней части
lr=long/2;%длина от середины до задней части
dr=weigh/2;
dl=weigh/2;
r=0.25/2;%радиус колес
m=70;%масса модели %100
J=228;%момент инерции
g=9.8;
%500Wt, 20km/h, 10', 100kg, 70x150x15
% коэффициенты сцепления (одинак для всех колес)(подобраны, чтоб было красиво)
mu=0.7;
Ca=10000;

P=m*g;

%параметры сервопривода рулевых колес
K=1;
T=0.1;

%параметры приводных двигаетлей
Un=36;
In=5;
wn=7000/60*6.28;
Reduct=5;
Mn=1.5;
L=0.1;
R=5;
Km=Mn/In;
Ke=(Un-In*R)/wn;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% usfr=pi/6;%задание поворота переднего колеса
% usfl=pi/6;%задание поворота переднего колеса
% usrr=pi/6;%задание поворота заднего колеса
% usrl=pi/6;%задание поворота заднего колеса
% urr=110;%напряжение питания заднего колеса
% ufr=110;%напряжение питания переднего колеса
% url=110;%напряжение питания заднего колеса
% ufl=110;%напряжение питания переднего колеса

%обобщенная сила сопротивления движению (сопр воздуха, прицеп и пр.)
Dax=0;
Day=0;

long_angle=0; %наклон дороги к продольной оси
lateral_angle=0; %наклон дороги к поперечной оси

%начальные условия
y=[0;0;0;0;0;0;0;0;0;0;0;0;0;0];
dt=0.001;
ym=[];
Vx_=0; Vy_=0; w_=0; Vxfr=0;Vxfl=0;Vxrr=0;Vxrl=0;
xg=0;
yg=3;
x0=0;
y0=0;
u=110;
t=[];
tm=0;% текущее премя
Fy=[];
asl=[];
% выход из цикла, если приблизились к точке/остановились/цончилось время
dtCount=100;
fileID = fopen('commands.txt','w');
for yg=[2.5 5 2.5 0]
while (true) 
if (tm>30) break; end
cfrr=y(1);%сила тока якоре
cffr=y(2);
x0=y(3);%текущая координата ц.м.
y0=y(4);
Vx=y(5);%скорость в координатах связанных с моделью
Vy=y(6);
w=y(7);%угловая скорость
gamfr=y(8);%угол поворота переднего колеса
gamrr=y(9);%угол поворота заднего колеса
fi=y(10);%угол поворота модели в глобальной СК
cfrl=y(11);%сила тока якоре
cffl=y(12);
gamfl=y(13);%угол поворота переднего колеса
gamrl=y(14);%угол поворота заднего колеса

% gamfr=usfr;
% gamrr=usrr;
% gamfl=usfl;
% gamrl=usrl;


%регулятор
if (dtCount>=10)
    ex=xg-x0;
    ey=yg-y0;
    u=sqrt(ex^2+ey^2)*3;
    if sqrt(ex^2+ey^2)<0.05
        u=0;
    end
    if (sqrt(ex^2+ey^2)<0.05) break; end
    if (u>36) 
        u=36;
    end
    urr=u;
    url=u;
    ufr=u;
    ufl=u;
    tt_=atan2(ey,ex);
    gam=(tt_-fi);
    if abs(gam)>pi
        gam=-sign(gam)*2*pi+gam;
    end
    if abs(gam)>pi/4
        gam=pi/4*sign(gam);
    end
    % usfr=gam;
    % usfl=gam;
    % usrr=-gam;
    % usrl=-gam;
    [usfl,usfr,usrl,usrr,ufl,ufr,url,urr]=kinem(gam, u,fi);
    dtCount=0;
    fprintf(fileID,'%5.0f;%5.0f\n',round(u*100),round(gam*100));
else
    dtCount=dtCount+1;
end
%распределение весов на колеса
Pf=(P*lr);%*cos(long_angle)-P/g*Vx_*heigh-Dax*heigh-P*heigh*sin(long_angle))/long;
Pr=(P*lf);%*cos(long_angle)+P/g*Vx_*heigh+Dax*heigh+P*heigh*sin(long_angle))/long;

Pfl=(Pf*dr);%*cos(lateral_angle)-Pf/g*Vy_*heigh-Day*heigh-Pf*heigh*sin(lateral_angle))/weigh;
Pfr=(Pf*dl);%*cos(lateral_angle)+Pf/g*Vy_*heigh+Day*heigh+Pf*heigh*sin(lateral_angle))/weigh;
Prl=(Pr*dr);%*cos(lateral_angle)-Pr/g*Vy_*heigh-Day*heigh-Pr*heigh*sin(lateral_angle))/weigh;
Prr=(Pr*dl);%*cos(lateral_angle)+Pr/g*Vy_*heigh+Day*heigh+Pr*heigh*sin(lateral_angle))/weigh;
if (Pfl<0 || Pfr<0 || Prl<0 || Prr<0) %моделирование заканчивается при отрыве любого колеса от земли
    disp('break');
    break;
end
Pa=Pfl+Pfr+Prr+Prl;
%скорости колес в связанных с ними СК
Vxfr1=Vxfr;
Vxfl1=Vxfl;
Vxrr1=Vxrr;
Vxrl1=Vxrl;

Vxfl=(Vx-dl*w)*cos(gamfl)+(Vy+lf*w)*sin(gamfl);
Vyfl=-(Vx-dl*w)*sin(gamfl)+(Vy+lf*w)*cos(gamfl);
Vxfr=(Vx+dr*w)*cos(gamfr)+(Vy+lf*w)*sin(gamfr);
Vyfr=-(Vx+dr*w)*sin(gamfr)+(Vy+lf*w)*cos(gamfr);
Vxrl=(Vx-dl*w)*cos(gamrl)+(Vy-lr*w)*sin(gamrl);
Vyrl=-(Vx-dl*w)*sin(gamrl)+(Vy-lr*w)*cos(gamrl);
Vxrr=(Vx+dr*w)*cos(gamrr)+(Vy-lr*w)*sin(gamrr);
Vyrr=-(Vx+dr*w)*sin(gamrr)+(Vy-lr*w)*cos(gamrr);

Vxfl_=(Vxfl-Vxfl1)/dt;
Vxfr_=(Vxfr-Vxfr1)/dt;
Vxrl_=(Vxrl-Vxrl1)/dt;
Vxrr_=(Vxrr-Vxrr1)/dt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%% модели двигателей
%задний правый
Mrr=Km*cfrr;
err=Ke*Vxrr/r*Reduct;
cfrr_=(urr-err-R*cfrr)/L;
%передний правый
Mfr=Km*cffr;
efr=Ke*Vxfr/r*Reduct;
cffr_=(ufr-efr-R*cffr)/L;
%задний левый
Mrl=Km*cfrl;
erl=Ke*Vxrl/r*Reduct;
cfrl_=(url-erl-R*cfrl)/L;
%передний левый
Mfl=Km*cffl;
efl=Ke*Vxfl/r*Reduct;
cffl_=(ufl-efl-R*cffl)/L;

Fxfr=Mfr/r;%движуие силы
Fxrr=Mrr/r;
Fxfl=Mfl/r;
Fxrl=Mrl/r;
%поперечные силы,действующие на колеса
alfafr=atan2(Vyfr,Vxfr);
alfarr=atan2(Vyrr,Vxrr);
alfafl=atan2(Vyfl,Vxfl);
alfarl=atan2(Vyrl,Vxrl);

Fy=Vy_*m;
Ca=atan2(Fy,atan2(Vx,Vy));

tfr=2*(Ca*Pfr/3/100000)/3/mu/Pfr;
aslfr=atan(tfr);
Lfr=1-tfr*abs(tan(alfafr));
if (abs(alfafr)<(aslfr))
    Fyfr=mu*Pfr*(1-Lfr^3)*sign(alfafr);
else
    Fyfr=-mu*Pfr*sign(alfafr);
end

trr=2*(Ca*Prr/3/100000)/3/mu/Prr;
aslrr=atan(trr);
Lrr=1-trr*abs(tan(alfarr));
if (abs(alfarr)<(aslrr))
    Fyrr=mu*Prr*(1-Lrr^3)*sign(alfarr);
else
    Fyrr=-mu*Prr*sign(alfarr);
end

tfl=2*(Ca*Pfl/3/100000)/3/mu/Pfl;
aslfl=atan(tfl);
Lfl=1-tfl*abs(tan(alfafl));
if (abs(alfafl)<(aslfl))
    Fyfl=mu*Pfl*(1-Lfl^3)*sign(alfafl);
else
    Fyfl=-mu*Pfl*sign(alfafl);
end

trl=2*(Ca*Prl/3/100000)/3/mu/Prl;
aslrl=atan(trl);
Lrl=1-trl*abs(tan(alfarl));
if (abs(alfarl)<(aslrl))
    Fyrl=mu*Prl*(1-Lrl^3)*sign(alfarl);
else
    Fyrl=-mu*Prl*sign(alfarl);
end

%движение ц.м.
XG_=cos(fi)*Vx-sin(fi)*Vy;
YG_=sin(fi)*Vx+cos(fi)*Vy;
Vx_=(Fxfr*cos(gamfr)+Fxrr*cos(gamrr)+Fxfl*cos(gamfl)+Fxrl*cos(gamrl)...
    +Fyrr*sin(gamrr)-Fyfr*sin(gamfr)+Fyrl*sin(gamrl)-Fyfl*sin(gamfl)-P*sin(long_angle))/m+w*Vy;  
Vy_=(-Fxrr*sin(gamrr)+Fxfr*sin(gamfr)-Fxrl*sin(gamrl)+Fxfl*sin(gamfl)...
    +Fyrr*cos(gamrr)+Fyfr*cos(gamfr)+Fyrl*cos(gamrl)+Fyfl*cos(gamfl)-P*sin(lateral_angle))/m-w*Vx; 
w_=(Fxrl*(lr*sin(gamrl)-dl*cos(gamrl))+Fyrl*(-lr*cos(gamrl)-dl*sin(gamrl))...
    +Fxrr*(lr*sin(gamrr)+dr*cos(gamrr))+Fyrr*(-lr*cos(gamrr)+dr*sin(gamrr))...
    +Fxfl*(lf*sin(gamfl)-dl*cos(gamfl))+Fyfl*(lf*cos(gamfl)+dl*sin(gamfl))...
    +Fxfr*(lf*sin(gamfr)+dr*cos(gamfr))+Fyfr*(lf*cos(gamfr)-dr*sin(gamfr)))/J;
gamfr_=-(1/T)*gamfr+(K/T)*usfr;
gamrr_=-(1/T)*gamrr+(K/T)*usrr;
gamfl_=-(1/T)*gamfl+(K/T)*usfl;
gamrl_=-(1/T)*gamrl+(K/T)*usrl;
%ДУ решается методом Эйлера 1-го пор. без солверов
dydt=[cfrr_; cffr_; XG_; YG_; Vx_; Vy_; w_; gamfr_; gamrr_; w;cfrl_; cffl_; gamfl_; gamrl_];
y=y+dt*dydt;
ym=[ym y];
tm=tm+dt;
t=[t tm];
end
end
%красивый вывод
t=t';
ym=ym';
figure(1)
subplot(2,2,1)
plot(ym(:,3), ym(:,4));
hold on
quiver(ym(1:500:end,3),ym(1:500:end,4),...
    (0.01*cos(ym(1:500:end,10))),(0.01*sin(ym(1:500:end,10))));
xlabel('x,м')
ylabel('y,м')
title('x,y');
grid on
subplot(2,2,2)
plot(t, ym(:,5))
xlabel('t,c')
ylabel('Vx,м/s')
title('Vx');
grid on
subplot(2,2,3)
plot(t, ym(:,10))
xlabel('t,c')
ylabel('fi,rad')
title('fi');
grid on
subplot(2,2,4)
plot(t, ym(:,6))
xlabel('t,c')
ylabel('Vy,м/s')
title('Vy');
grid on
function [gfl,gfr,grl,grr,Vfl,Vfr,Vrl,Vrr]=kinem(gam, V,gam0)
%gam0=0;%pi/6;
% V=5.250;
% R=-3.716;
gam0=0;
L=0.94;
C=0.63;
R=L/2/tan(gam);
% gam=atan2(L/2*cos(gam0),R);
if (abs(R)==inf) R=sign(R)*999999; end
% xf=L/2*cos(gam0);
% yf=L/2*sin(gam0);
% xr=L/2*cos(gam0+pi);
% yr=L/2*sin(gam0+pi);
x=R*cos(pi/2+gam0);
y=R*sin(pi/2+gam0);
% plot([x xf xr y],[y yf yr -x],'*')
if (R>0)
    gfl=atan2((L/2-x),(y-C/2));
    gfr=atan2((L/2-x),(y+C/2));
    grl=atan2((-L/2-x),(y-C/2));
    grr=atan2((-L/2-x),(y+C/2));
else
    grr=atan2((L/2+x),(-y-C/2));
    grl=atan2((L/2+x),(-y+C/2));
    gfr=atan2((-L/2+x),(-y-C/2));
    gfl=atan2((-L/2+x),(-y+C/2));
end
Rfl=abs(y-C/2)/abs(cos(gfl));
Rfr=abs(y+C/2)/abs(cos(gfr));
Rrl=abs(y-C/2)/abs(cos(grl));
Rrr=abs(y+C/2)/abs(cos(grr));
Rm=max([Rfl Rfr Rrl Rrr]);
Vfl=V*Rfl/Rm;
Vfr=V*Rfr/Rm;
Vrl=V*Rrl/Rm;
Vrr=V*Rrr/Rm;
end