
close all;
clear all;
clc;




global DELTA_ANGLE  TIME_PER_DEGREE 

global AMULTIPLIER  BMULTIPLIER  CMULTIPLIER 


global FORWARD  BACKWARD STOP    

global AX AY BX BY CX CY     

global A_INIT B_INIT C_INIT   

global F_A_THETA F_B_THETA F_C_THETA  

global W_A_THETA  W_B_THETA W_C_THETA  

global RATE_SAT AEB BEB CEB   


global beacon1 beacon2 beacon3

% last 3 time's data of time and angle for beacon A when it hit rover
global aLastTime2 aLastTime1 aTime aLastEst2 aLastEst1 aThetaEst 

%last 3 time's data of time and angle for beacon B when it hit rover
global bLastTime2 bLastTime1 bTime bLastEst2 bLastEst1 bThetaEst 


%last 3 time's data of time and angle for beacon C when it hit rover
global cLastTime2 cLastTime1 cTime cLastEst2 cLastEst1 cThetaEst 

%rate of change of theta for A, B, C (while it hits rover)
global aRateL aRate bRateL bRate cRateL cRate 

%last 2 positions of rover
global lastpositionLx lastpositionx lastpositionLy lastpositiony 

%estimated angle
global bEst cEst aEst 

%current position of rover and for its calculation
global px py px1 py1 px2 py2 

%distance between beacons and helping variables
global h1 h2 PA PC 

%target location, target's angle from rover and rover's rotation time
global tAngle targetx targety delaAngle rotaTime 

%for startup
global startTime startupposition1x startupposition1y startupposition2x startupposition2y hAngle

%counting for beacon identification
global  count count2

% rotational direction of rover
global direction


%check for some specific location

global cond2 cond 

%confirmed signal from beacon A, B, C
global beaconAhit beaconBhit beaconChit 

%for startup
global startuprover firstStartup

%beacon availability
global beaconAavailable beaconBavailable beaconCavailable

%beacons identified
global identifiedA identifiedB identifiedC 

%rover modes
global  rovergo rotatting 

%positionLast
global positionLastx positionLasty 

%angleTime
global angleTime


%testing
%global tA tB tC

% global taCount tbCount tcCount


DELTA_ANGLE   =      0.144 ;       % degrees/millisecond changes during rotation
TIME_PER_DEGREE=     6.944445 ;   % time/degree change during rotation

% angle change in degree per milisecond while beacon frequency is 1 rotation/second
AMULTIPLIER = 0.36;  
BMULTIPLIER = 0.36;
CMULTIPLIER = 0.36;


FORWARD         =    1;         
BACKWARD        =    2;         
STOP            =    3;         

AX              =    0;
AY              =    0;           

BX              =   1000;
BY              =    0;

CX              =    1000;
CY              =    1000;

A_INIT          =    0;
B_INIT          =    90;
C_INIT          =    180;

F_A_THETA       =    1;
F_B_THETA       =    2;
F_C_THETA       =    3;  


W_A_THETA    =       1000/F_A_THETA;      %    
W_B_THETA    =       1000/F_B_THETA;        
W_C_THETA    =       1000/F_C_THETA;         


RATE_SAT      =      0;      

AEB           =      8;       
BEB           =      6;     
CEB           =      4;       


beacon1 =0;
beacon2 =0;
beacon3 =0;



% last 3 time's data of time and angle for beacon A when it hit rover
aLastTime2 = 0;
aLastTime1 = 0;
aTime = 0;
aLastEst2 = 0;
aLastEst1 = 0; 
aThetaEst = A_INIT; 

%last 3 time's data of time and angle for beacon B when it hit rover
bLastTime2 = 0;
bLastTime1 = 0;
bTime = 0;
bLastEst2 = 0;
bLastEst1 = 0;
bThetaEst = B_INIT;


%last 3 time's data of time and angle for beacon C when it hit rover
cLastTime2 = 0;
cLastTime1 = 0;
cTime = 0;
cLastEst2 = 0;
cLastEst1 = 0;
cThetaEst = C_INIT;

%rate of change of theta for A, B, C (while it hits rover)
aRateL = 0;
aRate = 0;
bRateL = 0; 
bRate = 0; 
cRateL = 0;
cRate = 0;

%last 2 positions of rover
lastpositionLx = 0;
lastpositionx = 0; 
lastpositionLy = 0;
lastpositiony =0;

%estimated angle
bEst = 0;
cEst = 0;
aEst = 0;

%current position of rover and for its calculation
px = 0; 
py = 0; 
px1 = 0; 
py1 = 0;
px2 = 0;
py2 = 0;

%distance between beacons and helping variables
h1 = 0;
h2 = 0;
PA = 0;
PC = 0;


hAngle2= 25 ;
target = [652,321];
initialPosition= [20,990];

%target location, target's angle from rover and rover's rotation time
tAngle=0;
targetx =target(1);
targety=target(2);
delaAngle=0;
rotaTime =0;


%for startup
startTime=0;
startupposition1x=0;
startupposition1y=0;
startupposition2x=0;
startupposition2y=0;
hAngle=0;

%positionLast
positionLastx=0;
positionLasty=0;
angleTime=0;


%counting for beacon identification
count=0;
count2 =0;

% rotational direction of rover
direction=0;

%rover wheel rotation
rightWheel = STOP;
leftWheel = STOP;

%check for some specific location
cond2 = false;
cond = false;

%confirmed signal from beacon A, B, C
beaconAhit = false;
beaconBhit = false;
beaconChit = false;

%for startup
startuprover=true;
firstStartup=true;

%beacon availability
beaconAavailable=false;
beaconBavailable=false;
beaconCavailable=false;

%beacons identified
identifiedA = false;
identifiedB = false;
identifiedC =false;

%rover modes
rovergo=false;
rotatting = false;

global AC AB BC

global bcaTheta cabTheta

global tA tB tC

global lastTLocation

global taCount tbCount tcCount
taCount=1;
tbCount=1;
tcCount=1;

lastTLocation=0;
%testing
tA=[];
tB=[];
tC=[];
    
beaconAX=AX;
beaconAY=AY;
beaconBX=BX;
beaconBY=BY;
beaconCX=CX;
beaconCY=CY;

AC = sqrt(((beaconCX - beaconAX)*(beaconCX - beaconAX))...
        + ((beaconCY - beaconAY)*(beaconCY - beaconAY)));
AB = sqrt(((beaconAX - beaconBX)*(beaconAX - beaconBX))...
        + ((beaconAY - beaconBY)*(beaconAY - beaconBY)));
BC = sqrt(((beaconCX - beaconBX)*(beaconCX - beaconBX))...
    + ((beaconCY - beaconBY)*(beaconCY - beaconBY)));

cabTheta = atandeg((beaconCY - beaconAY) / (beaconCX - beaconAX)); %// angle CAB
bcaTheta = 180 + cabTheta;                     % // angle BCA



%%-----------
dist2wheel=10;
rr= 5.9/2; %cm
rl= 5.9/2; %cm
fL=1.447;
fR=1.447;
omegaL= 2*3.14*fL;
omegaR= 2*3.14*fR;


dATheta = (F_A_THETA*360)/3600;
dBTheta = (F_B_THETA*360)/3600;
dCTheta = (F_C_THETA*360)/3600;



hAngle =0;

%[x,y]=meshgrid(0:0.01:100,0:0.01:100);

myTable=[];
aTable=[];
bTable=[];
cTable=[];
gpp=0;

errLimit=10;
mysat=0.5;



        
Rox= initialPosition(1);
Roy= initialPosition(2);
ini=[Rox,Roy];

aTheta = A_INIT;
bTheta = B_INIT;
cTheta = C_INIT;

rvsign=1;
lvsign=1;
original=ini;
originalPath=[];
detectedPath=[];

%figure
t1=0;
for i=1:(40*3600)
    lastoriginal=original;
    original=[Rox,Roy];
    lastT=t1;
    t1=t1+0.27777777777777777777777778;
 
    
    aTheta = aTheta + dATheta;
    aTheta=round(aTheta*100)/100;
    aTheta=mod(aTheta,360);
    mygop=0;
    if(AY <= Roy)
        h=acosd(-(AX - Rox)/(sqrt(((AX- Rox)^2) + ((AY-Roy)^2))));
    else 
        h=360-acosd(-(AX - Rox)/(sqrt(((AX- Rox)^2) + ((AY-Roy)^2))));
    end
    if ((aTheta-0.05) <= h) && (h <(aTheta+0.05))
        at=(aTheta-h)*0.27777777777777777777777778/0.1;
        myTable=[myTable;t1-at];
        aTable=[aTable;[t1-at,mod(A_INIT + (t1-at)*AMULTIPLIER*F_A_THETA,360)]];

        [LWR,RWR]=location(t1-at);
        if (~firstStartup)
            originalPath=[originalPath;[Rox,Roy]];
            detectedPath =[detectedPath;[px,py]];    
        end
    end

    bTheta = bTheta + dBTheta;
    bTheta=round(bTheta*100)/100;
    bTheta=mod(bTheta,360);
  
    if(BY <= Roy)
        h=acosd(-(BX - Rox)/(sqrt(((BX- Rox)^2) + ((BY-Roy)^2))));
    else 
        h=360-acosd(-(BX - Rox)/(sqrt(((BX- Rox)^2) + ((BY-Roy)^2))));
    end

    if(((bTheta-0.1) <= h) && (h <(bTheta+0.1)))
        bt=(bTheta-h)*0.27777777777777777777777778/0.2;
        myTable=[myTable;t1-bt];
        bTable=[bTable;[t1-bt, mod(B_INIT + (t1-bt)*BMULTIPLIER*F_B_THETA,360)]];
        [LWR,RWR]=location(t1-bt);
        if (~firstStartup)
            originalPath=[originalPath;[Rox,Roy]];
            detectedPath =[detectedPath;[px,py]];    
        end
    end
    
    
    cTheta = cTheta + dCTheta;
    cTheta=round(cTheta*100)/100;
    cTheta=mod(cTheta,360);
    if(CY <= Roy)
        h=acosd(-(CX - Rox)/(sqrt(((CX- Rox)^2) + ((CY-Roy)^2))));
    else 
        h=360-acosd(-(CX - Rox)/(sqrt(((CX- Rox)^2) + ((CY-Roy)^2))));
    end
    if(((cTheta-0.15) <= h) && (h <(cTheta+0.15)))
        ct=(cTheta-h)*0.27777777777777777777777778/0.3;
        myTable=[myTable;t1-ct];
        cTable=[cTable;[t1-ct, mod(C_INIT + (t1-ct)*CMULTIPLIER*F_C_THETA,360)]];
        
        [LWR,RWR]=location(t1-ct);
        if (~firstStartup)
            originalPath=[originalPath;[Rox,Roy]];
            detectedPath =[detectedPath;[px,py]];    
        end
    end
    
    roverposition= createMarker(original,hAngle2);
    roverlocation= createMarker([px,py],hAngle2);
    if (mod(i,720)==0)% && mod(i,600)==0)
        plot([AX,BX,CX,AX,AX],[AY,BY,CY,CY,AY],'-b')
        hold on
        plot([AX,BX,CX],[AY,BY,CY],'or','MarkerFaceColor','r')
        fill(roverposition(1,:),roverposition(2,:),'YELLOW')
        plot(roverposition(1,1),roverposition(2,1),'yo','MarkerFaceColor','y','MarkerSize',4)
        
        plot(targetx,targety,'sg','MarkerFaceColor','g');
       
        fill(roverlocation(1,:),roverlocation(2,:),'Blue')
        plot(roverlocation(1,1),roverlocation(2,1),'bo','MarkerFaceColor','b','MarkerSize',4)
        % plot (px,py,'sr','MarkerFaceColor','r')
   
        plot([AX ,(AX+1450*cosd(aTheta))],[AY ,(AY+1450*sind(aTheta))],'--r')
        plot([CX ,(CX+1450*cosd(cTheta))],[CY ,(CY+1450*sind(cTheta))],'--r')
        plot([BX ,(BX+1450*cosd(bTheta))],[BY ,(BY+1450*sind(bTheta))],'--r')
              
        axis([-100 1200 -100 1200])
        
        Min=floor(t1/60000);
        left1=t1-(Min*60000);
        seconds=floor(left1/1000);
        miliSec= (left1-seconds*1000);
        if Min<10
            strm=['0' num2str(Min)];
        else
            strm=num2str(Min);
        end
        if seconds<10
            strs=[':0' num2str(seconds)];
        else
            strs=[':' num2str(seconds)];
        end
        strms=[':' num2str(miliSec)];
        strt=['Time:' strm strs strms]; 
        text(-5,105,strt,'fontsize',12,'fontweight','bold')
        
        hold off
        drawnow
    end
  
    velo= ((rl*omegaL)+ (rr*omegaR))/(2*1000);
    
    if ((~rotatting)  && (rovergo))
        dvelox= velo*cosd(hAngle2)*(t1-lastT);
        dveloy= velo*sind(hAngle2)*(t1-lastT);
    
        Rox= Rox + dvelox ;
        Roy= Roy + dveloy ;
    end
    if ((rotatting)&& (~rovergo))
        hAngle2= hAngle2 + direction*DELTA_ANGLE *(t1-lastT);
        rotaTime=rotaTime-(t1-lastT);
        if rotaTime<0
            rotaTime=0;
            rotatting=0;
            rovergo=1;
        end
    end
end

figure
plot(originalPath(:,1),originalPath(:,2),'-r')
hold on
plot(detectedPath(:,1),detectedPath(:,2),'-k')
plot(targetx,targety,'sg','MarkerFaceColor','g');
plot(initialPosition(1),initialPosition(2),'sr','MarkerFaceColor','r');
text(initialPosition(1)-10,initialPosition(2)-10,'Start',...
    'fontsize',12,'fontweight','bold');
text(targetx-10,targety-10,'Target',...
    'fontsize',12,'fontweight','bold');

legend('original', 'sensor')
title('detected and original paths of rover')

%%        



function [rightWheel,leftWheel]=location (myTable)   


global DELTA_ANGLE  TIME_PER_DEGREE 

global AMULTIPLIER  BMULTIPLIER  CMULTIPLIER 

global FORWARD  BACKWARD STOP    

global AX AY BX BY CX CY     

global A_INIT B_INIT C_INIT   

global F_A_THETA F_B_THETA F_C_THETA  

global W_A_THETA  W_B_THETA W_C_THETA  

global RATE_SAT AEB BEB CEB   

global beacon1 beacon2 beacon3

% last 3 time's data of time and angle for beacon A when it hit rover
global aLastTime2 aLastTime1 aTime aLastEst2 aLastEst1 aThetaEst 

%last 3 time's data of time and angle for beacon B when it hit rover
global bLastTime2 bLastTime1 bTime bLastEst2 bLastEst1 bThetaEst 


%last 3 time's data of time and angle for beacon C when it hit rover
global cLastTime2 cLastTime1 cTime cLastEst2 cLastEst1 cThetaEst 

%rate of change of theta for A, B, C (while it hits rover)
global aRateL aRate bRateL bRate cRateL cRate ;

%last 2 positions of rover
%global lastpositionLx lastpositionx lastpositionLy lastpositiony 

%estimated angle
global bEst cEst aEst 

%current position of rover and for its calculation
global px py px1 py1 px2 py2 

%distance between beacons and helping variables
global h1 h2 PA PC 

%target location, target's angle from rover and rover's rotation time
global tAngle targetx targety delaAngle rotaTime 

%for startup
global startTime startupposition1x startupposition1y startupposition2x startupposition2y hAngle

%counting for beacon identification
global  count count2

% rotational direction of rover
global direction

%check for some specific location

global  cond2 cond 

%confirmed signal from beacon A, B, C
global beaconAhit beaconBhit beaconChit 

%for startup
global startuprover firstStartup

%beacon availability
global beaconAavailable beaconBavailable beaconCavailable

%beacons identified
global identifiedA identifiedB identifiedC 

%rover modes
global  rovergo rotatting 

%positionLast
global positionLastx positionLasty

global angleTime

%testing
global tA tB tC

global taCount tbCount tcCount

global lastTLocation

global AC AB BC

global bcaTheta cabTheta

distance =1;
gop1234=1
                   
   if (count ==2)
   
        beacon3 = myTable;
        count=3;
   end

    if (count ==1)
    
        beacon2=myTable;
        count=2;
    end

    if (count ==0)
    
        beacon1= myTable;
        count=1;
    end

        
    
    if (count>1 && (~beaconAavailable ||~beaconBavailable ||  ~beaconCavailable ))  
        if (((myTable <= (beacon1 +W_A_THETA + AEB)) && (myTable >= (beacon1 +W_A_THETA - AEB)))...
                || ((myTable <= (beacon2 +W_A_THETA + AEB)) && (myTable >= (beacon2 +W_A_THETA - AEB)))...
                || ((myTable <= (beacon3 +W_A_THETA + AEB)) && (myTable >= (beacon3 +W_A_THETA - AEB)))) % assign
            aTime= myTable;
            identifiedA = true;
            tA(taCount)=aTime;
            taCount=taCount+1;
        end

        if (((myTable <= (beacon1 +W_B_THETA + BEB)) && (myTable >= (beacon1 +W_B_THETA - BEB)))||...
                ((myTable <= (beacon2 +W_B_THETA + BEB)) && (myTable >= (beacon2 +W_B_THETA - BEB)))...
                || ((myTable <= (beacon3 +W_B_THETA + BEB)) && (myTable >= (beacon3 +W_B_THETA - BEB))))% assign
            bTime= myTable;
            identifiedB =true;
            tB(tbCount)=bTime;
            tbCount=tbCount+1;
        end
        if (((myTable <= (beacon1 +W_C_THETA + CEB)) && (myTable >= (beacon1 +W_C_THETA - CEB)))...
                || ((myTable <= (beacon2 +W_C_THETA + CEB)) && (myTable >= (beacon2 +W_C_THETA - CEB)))...
                || ((myTable <= (beacon3 +W_C_THETA + CEB)) && (myTable >= (beacon3 +W_C_THETA - CEB)))) %assign
            cTime= myTable;
            identifiedC =true;
            tC(tcCount)=cTime;
            tcCount=tcCount+1;
        end
        

        if (identifiedA)
            if ((myTable <= (aTime + W_A_THETA +AEB)) && (myTable >= (aTime + W_A_THETA - AEB)))
                aLastTime1 = aTime;
                aTime= myTable;
                aLastEst1 = mod(A_INIT + aLastTime1 * AMULTIPLIER * F_A_THETA,360);
                aThetaEst = mod(A_INIT + aTime * AMULTIPLIER * F_A_THETA,360);
                aRate = (aThetaEst - aLastEst1) / (aTime - aLastTime1);
                beaconAavailable =true;
                tA(taCount)=aTime;
                taCount=taCount+1;
            end
        end

        if (identifiedB)
            if ((myTable <= (bTime + W_B_THETA +BEB)) && (myTable >= (bTime + W_B_THETA - BEB)))
                bLastTime1 = bTime;
                bTime= myTable;
                bLastEst1 = mod(B_INIT + bLastTime1 * BMULTIPLIER * F_B_THETA,360);
                bThetaEst = mod(B_INIT + bTime * BMULTIPLIER* F_B_THETA,360);
                bRate = (bThetaEst - bLastEst1) / (bTime - bLastTime1);
                beaconBavailable =true;
                tB(tbCount)=bTime;
                tbCount=tbCount+1;
            end
        end

        if (identifiedC)
            if ((myTable <= (cTime + W_C_THETA +CEB)) && (myTable >= (cTime + W_C_THETA - CEB)))
                cLastTime1 = cTime;
                cTime= myTable;
                cLastEst1 = mod(C_INIT + cLastTime1 * CMULTIPLIER* F_C_THETA,360);
                cThetaEst = mod(C_INIT + cTime * CMULTIPLIER * F_C_THETA,360);
                cRate = (cThetaEst - cLastEst1) / (cTime - cLastTime1);
                beaconCavailable =true;
                tC(tcCount)=cTime;
                tcCount=tcCount+1;
            end
        end
    end

    
    if (beaconAavailable && beaconBavailable && beaconCavailable) 
        if ((myTable <= (aTime + W_A_THETA +AEB)) && (myTable >= (aTime + W_A_THETA - AEB)))
            aLastTime2 = aLastTime1;
            aLastTime1 = aTime;
            aTime= myTable;
            aLastEst2 = aLastEst1;
            aLastEst1 = aThetaEst;
            aThetaEst = mod(A_INIT + aTime*AMULTIPLIER*F_A_THETA,360);
            aRateL = aRate;
            aRate = (aThetaEst - aLastEst1) / (aTime - aLastTime1);
            if (abs(aRate - aRateL) > RATE_SAT)
                aRate = (aThetaEst - aLastEst2) / (aTime - aLastTime2);
            end
                tA(taCount)=aTime;
                taCount=taCount+1;
                beaconAhit =true;
        end
        if ((myTable <= (bTime + W_B_THETA +BEB)) && (myTable >= (bTime + W_B_THETA - BEB)))
            bLastTime2 = bLastTime1;
            bLastTime1 = bTime;
            bTime = myTable;
            bLastEst2 = bLastEst1;
            bLastEst1 = bThetaEst;
            bThetaEst = mod(B_INIT + bTime * BMULTIPLIER * F_B_THETA,360);
            bRateL = bRate;
            bRate = (bThetaEst - bLastEst1) / (bTime - bLastTime1);
            if (abs(bRate - bRateL) > RATE_SAT)
                bRate = (bThetaEst - bLastEst2) / (bTime - bLastTime2);
            end
            beaconBhit =true;
            tB(tbCount)=bTime;
            tbCount=tbCount+1;
        end
        if ((myTable <= (cTime + W_C_THETA +CEB)) && (myTable >= (cTime + W_C_THETA - CEB)))
            cLastTime2 = cLastTime1;
            cLastTime1 = cTime;
            cTime = myTable;
            cLastEst2 = cLastEst1;
            cLastEst1 = cThetaEst;
            cThetaEst =mod(C_INIT + cTime * CMULTIPLIER * F_C_THETA,360);
            cRateL = cRate;
            cRate = (cThetaEst - cLastEst1) / (cTime - cLastTime1);
            if (abs(cRate - cRateL) > RATE_SAT)
                cRate = (cThetaEst - cLastEst2) / (cTime - cLastTime2);
            end
            beaconChit =true;
            tC(tcCount)=cTime;
            tcCount=tcCount+1;
        end
    end


    if px<600 && px>0
        gkjkjfkg=09;
    end
    if (beaconAhit)
        beaconAhit=false; 
        bEst = mod((bThetaEst + bRate * (aTime - bTime)), 360);
        cEst = mod((cThetaEst + cRate * (aTime - cTime)),  360);
        if ((~((bEst <= 0.001 && bEst >= -0.001)||...
                (bEst <= 180 + 0.001 && bEst >= 180 - 0.001)||...
                (aThetaEst <= 0.001 && aThetaEst >= -0.001) ||...
                (aThetaEst <= 180 + 0.001 && aThetaEst >= 180 - 0.001)))...
                && beaconBavailable)
            h1 = AB / (1 + (tandeg(aThetaEst) / tandeg(180 - bEst)));
            px1 = AX + h1;
            py1 = AY + h1 * tandeg(aThetaEst);
            cond = true;
        end
        if ((~(((aThetaEst <= cabTheta + gop1234) && (aThetaEst >= cabTheta -gop1234))||...
                ((aThetaEst <= bcaTheta + gop1234) && (aThetaEst >= bcaTheta - gop1234)) ||...
                ((cEst <= cabTheta + gop1234) && (cEst >= cabTheta - gop1234)) ||...
                ((cEst <= bcaTheta + gop1234) && (cEst >= bcaTheta - gop1234))))...
                && beaconCavailable)
            h2 = (AC/ (1 + ((tandeg(360 + cabTheta - aThetaEst))/ tandeg(cEst - bcaTheta))));
            PA = abs(h2 / cosdeg(cabTheta - aThetaEst));
            px2 = AX + PA * cosdeg(aThetaEst);
            py2 = AY + PA * sindeg(aThetaEst);
            cond2 = true;
        end
        if cond
                if cond2
                    px= (px1 + px2)/2;
                    py= (py1 + py2)/2;
                else
                    px=px1;
                    py=py1;
                end
                cond=false;
                cond2=false;
            elseif cond2 
                px=px2;
                py=py2;
                cond2=false;
        end

        if (startuprover)
            if (firstStartup)
                startTime = myTable;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            end
            rovergo = true;
            if ((myTable-startTime)>2000)
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,startupposition1x, startupposition1y);                    
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                positionLastx= px;
                positionLasty= py; 
                angleTime=myTable;
                
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                end
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                end               
            end
            
  %--------- not immplemented in Rover's C code--------
%         elseif ((myTable-angleTime) >2000)
%             hAngle= anglefinder(px,py,positionLastx, positionLasty);                    
%             tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
%             delaAngle= hAngle- tAngle;
%             positionLastx= px;
%             positionLasty= py; 
%             angleTime=myTable;
%             
%             if (delaAngle > 180)
%                 delaAngle= delaAngle-360;
%             end
%             if (delaAngle < -180)
%                 delaAngle = delaAngle +360;
%             end
  %----------------------------------------------------
        end
        
        
        if (abs(delaAngle) > 5/(distance))
            rotatting=true;
            rovergo=false;
            rotaTime=TIME_PER_DEGREE *abs(delaAngle);
            angleTime= myTable+rotaTime; 
                    
            if (delaAngle < 0)
                direction =1;
            else
                direction =-1;     
            end  
            delaAngle=0;
        end
    end

    if(beaconBhit)
        beaconBhit = false;
        aEst = mod((aThetaEst + aRate * (bTime - aTime)),360);
        cEst = mod((cThetaEst + cRate * (bTime - cTime)),360);
        if ((~((bThetaEst <= 90 + 0.001 && bThetaEst >= 90 - 0.001)||...
                (bThetaEst <= 270 + 0.001 && bThetaEst >= 270 - 0.001)||...
                (cEst <= 90 + 0.001 && cEst >= 90 - 0.001) ||...
                (cEst <= 270 + 0.001 && cEst >= 270 - 0.001)))...
                && beaconCavailable)
            h1 = BC / (1 + (tandeg(bThetaEst - 90) / tandeg(270 - cEst)));
            py1 = BY + h1;
            px1 = BX - h1 * tandeg(bThetaEst - 90);
            cond = true;
        end
        if ((~((bThetaEst <= 0.001 && bThetaEst >= -0.001)||...
                (bThetaEst <= 180 + 0.001 && bThetaEst >= 180 - 0.001)||...
                (aEst <= 0.001 && aEst >= -0.001)||...
                (aEst <= 180 + 0.001 && aEst >= 180 - 0.001)))...
                && beaconAavailable)
            h2 = AB / (1 + (tandeg(180 - bThetaEst) / tandeg(aEst)));
            px2 = BX - h2;
            py2 = BY + h2 * tandeg(180 - bThetaEst);
            cond2 = true;
        end
         if cond
                if cond2
                    px= (px1 + px2)/2;
                    py= (py1 + py2)/2;
                else
                    px=px1;
                    py=py1;
                end
                cond=false;
                cond2=false;
            elseif cond2 
                px=px2;
                py=py2;
                cond2=false;
         end
        if (startuprover)
            if (firstStartup)
                startTime = myTable;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            end
            rovergo = true;
            if ((myTable-startTime)>2000)
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,startupposition1x, startupposition1y);                    
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                positionLastx= px;
                positionLasty= py; 
                angleTime=myTable;
                
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                end
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                end               
            end
  %--------- not immplemented in Rover's C code--------
%         elseif ((myTable-angleTime) >2000)
%             hAngle= anglefinder(px,py,positionLastx, positionLasty);                    
%             tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
%             delaAngle= hAngle- tAngle;
%             positionLastx= px;
%             positionLasty= py; 
%             angleTime=myTable;
%             
%             if (delaAngle > 180)
%                 delaAngle= delaAngle-360;
%             end
%             if (delaAngle < -180)
%                 delaAngle = delaAngle +360;
%             end
  %----------------------------------------------------
         end

        
        if (abs(delaAngle) > 5/(0.1*distance))
            rotatting=true;
            rovergo=false;
            rotaTime=TIME_PER_DEGREE *abs(delaAngle);
            angleTime= myTable+rotaTime; 
                    
            if (delaAngle < 0)
                direction =1;
            else
                direction =-1;     
            end  
            delaAngle=0;
        end
    end
    if (beaconChit)
        beaconChit =false;
        bEst = mod((bThetaEst + bRate * (cTime - bTime)),360);
        aEst = mod((aThetaEst + aRate * (cTime - aTime)),360);
        if ((~((bEst <= 90 + 0.001 && bEst >= 90 - 0.001)||...
                (bEst <= 270 + 0.001 && bEst >= 270 - 0.001) ||...
                (cThetaEst <= 90 + 0.001 && cThetaEst >= 90 - 0.001)||...
                (cThetaEst <= 270 + 0.001 && cThetaEst >= 270 - 0.001)))...
                && beaconBavailable)
            h1 = BC / (1 + (tandeg(270 - cThetaEst) / tandeg(bEst - 90)));
            py1 = CY - h1;
            px1 = CY - h1 * tandeg(270 - cThetaEst);
            cond = true;
        end
        if ((~(((aEst <= cabTheta + gop1234) && (aEst >= cabTheta - gop1234)) ||...
                ((aEst <= bcaTheta + gop1234) && (aEst >= bcaTheta - gop1234))||...
                ((cThetaEst <= cabTheta + gop1234) && (cThetaEst >= cabTheta - gop1234)) ||...
                ((cThetaEst <= bcaTheta + gop1234) && (cThetaEst >= bcaTheta - gop1234))))...
                && beaconAavailable)
            h2 = (AC/ (1+ ((tandeg(cThetaEst - bcaTheta))/ tandeg(360 + cabTheta - aEst))));
            PC = abs(h2 / cosdeg(cThetaEst - bcaTheta));
            px2 = CX + PC * cosdeg(cThetaEst);
            py2 = CY + PC * sindeg(cThetaEst);
            cond2 = true;
        end
        if cond
                if cond2
                    px= (px1 + px2)/2;
                    py= (py1 + py2)/2;
                else
                    px=px1;
                    py=py1;
                end
                cond=false;
                cond2=false;
       elseif cond2 
            px=px2;
            py=py2;
            cond2=false;
       else
            rovergo=false;
            rotatting =false;
       end

        if (startuprover)
            if (firstStartup)
                startTime = myTable;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            end
            rovergo = true;
            if ((myTable-startTime)>2000)
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,startupposition1x, startupposition1y);                    
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                positionLastx= px;
                positionLasty= py; 
                angleTime=myTable;
                
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                end
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                end               
            end
  %--------- not immplemented in Rover's C code--------
%         elseif ((myTable-angleTime) >2000)
%             hAngle= anglefinder(px,py,positionLastx, positionLasty);                    
%             tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
%             delaAngle= hAngle- tAngle;
%             positionLastx= px;
%             positionLasty= py; 
%             angleTime=myTable;
%             
%             if (delaAngle > 180)
%                 delaAngle= delaAngle-360;
%             end
%             if (delaAngle < -180)
%                 delaAngle = delaAngle +360;
%             end
  %----------------------------------------------------
         end
        
        
        
        if (abs(delaAngle) > 5/(distance))
            rotatting=true;
            rovergo=false;
            rotaTime=TIME_PER_DEGREE *abs(delaAngle);
            angleTime= myTable+rotaTime; 
                    
            if (delaAngle < 0)
                direction =1;
            else
                direction =-1;     
            end  
            delaAngle=0;
        end
        
    end

    if ((abs(px-targetx)<=5 )&& (abs(py-targety)<=5))
        rotatting=false;
        rovergo=false;
        rightWheel = STOP;
        leftWheel  = STOP;
    end
    if (rotatting)
        hAngle = hAngle + direction * DELTA_ANGLE*(myTable - lastTLocation);
%         rotaTime = rotaTime - (myTable - lastTLocation);
        if (rotaTime < 0)
            rotaTime = 0;
            rotatting = false;
            rovergo = true;
            direction=0;
        end
        if(direction==1)
            rightWheel= FORWARD;
            leftWheel = BACKWARD;
        else
            rightWheel = BACKWARD;
            leftWheel  = FORWARD;
        end
    end
    if (rovergo)
        rightWheel = FORWARD;
        leftWheel  = FORWARD;
    end
    if ((~rovergo) && (~rotatting))
        
        rightWheel = STOP;
        leftWheel  = STOP;
    end
    lastTLocation=myTable;
end
 
%%


function z= tandeg( y)
    z = tan(y*6.283/360);    % radian = Degree*2*pi/360    
end

%%
function z= atandeg( y)
    z = ((atan(y))*360/(2*3.14));    %// radian = Degree*2*pi/360
end

%%
function z= sindeg(y)
    z = sin(y*6.283/360);
end

%%
function z= cosdeg(y)
    z = cos(y*6.283/360);
end

%%
% find angle between two points
function theta1= anglefinder(  tx, ty, p_x, p_y)

    dx = tx - p_x;
    dy = ty - p_y;
    theta1 = abs(atandeg(dy/dx));
    if (dx < 0)
        if (dy < 0)
            theta1 = 180 + theta1;
        else
            theta1 = 180 - theta1;
        end
    else
        if (dy < 0)
            theta1 = 360 - theta1;
        end
    end
end

%%
function markerCoordinantes=createMarker(currentPoint,thetaTarget)
    

    % represent angle from -180 to 180 degree
    if thetaTarget>180
        thetaTarget= thetaTarget-360;
    elseif thetaTarget<-180
        thetaTarget= 360+thetaTarget;
    end

    a=25; %vertex's distsnce from centroid of triangle

    %Vertices of triangular Marker
    A=[currentPoint(1)+a*cosd(thetaTarget)   ,currentPoint(2)+a*sind(thetaTarget)];
    B=[currentPoint(1)-a*cosd(thetaTarget+60),currentPoint(2)-a*sind(thetaTarget+60)];
    C=[currentPoint(1)-a*cosd(60-thetaTarget),currentPoint(2)+a*sind(60-thetaTarget)];
    markerCoordinantes = [A(1) B(1) C(1) A(1) ; A(2) B(2) C(2) A(2)];

end