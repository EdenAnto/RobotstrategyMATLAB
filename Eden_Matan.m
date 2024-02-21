function [move,mem] = Eden_Matan(env,mem)
%Strategy for robot tournament game, following opponent
%
%Environment Struct
% field:
% info,  STRUCT{team, fuel, myPos, oppPos}
% basic, STRUCT{walls, rRbt, rMF, lmax}
% mines, STRUCT{nMine, mPos, mineScr, mineExist}
% fuels, STRUCT{nFuel, fPos, fuelScr, fuelExist}
%
%Memory Struct
% field:
% path, STRUCT{start, dest, pathpt, nPt, proc, lv}

fuels = env.fuels.fPos;
isFuelsExist = env.fuels.fExist;
existFuels =fuels(find(isFuelsExist==1),:);%%%%%%%%%%%%%
mypos=env.info.myPos;
opPos=env.info.opPos;
lmax=env.basic.lmax;
roboR=env.basic.rRbt;

targetFuel=findMeFuel(mypos,existFuels,opPos);

if (targetFuel==-1)
    move = [0 0];
    return;
end
attack=Attack(mypos,opPos,roboR,env,lmax);
if(attack)
    dx = opPos(1)-mypos(1);
    m= polyfit([opPos(1) mypos(1)],[opPos(2) mypos(2)],1);
    [moveX moveY]=DirectMove(lmax,m,dx);
    move= [moveX moveY];
    return;
end
dx = targetFuel(1)-mypos(1);
m= polyfit([targetFuel(1) mypos(1)],[targetFuel(2) mypos(2)],1);
[moveX moveY] = DirectMove(lmax,m,dx);



mines = env.mines.mPos;
isMinesExist=env.mines.mExist;
existMines =mines(find(isMinesExist==1),:);

dMines = distance(mypos,existMines);
closestMinesIdx = find(min(dMines)==dMines);
closestMines= existMines(closestMinesIdx,:);


mFRaidus = env.basic.rMF;
[flag loc] = inKav(existMines,mFRaidus,m,mypos,targetFuel,dx,lmax);
if (flag)
[nextX nextY]= mainFunc(flag,mFRaidus,lmax, mypos, targetFuel,existMines,moveX,moveY);
nextPos= [mypos(1)+nextX mypos(2)+nextY];
[tmp1X tmp1Y]= mainFunc(flag,mFRaidus,lmax, nextPos, targetFuel,existMines,moveX,moveY);
newPos= [nextPos(1)+tmp1X nextPos(2)+tmp1Y];
[tmp2X tmp2Y]= mainFunc(flag,mFRaidus,lmax, newPos, targetFuel,existMines,moveX,moveY);

newNewPos=[newPos(1)+tmp2X newPos(2)+tmp2Y];
if (abs(newNewPos(1)-nextPos(1))<0.15 && abs(newNewPos(2)-nextPos(2))<0.15)
good=1;    
else
    moveX=tmp1X;
    moveY=tmp1Y;
end
end



if (mypos(2)+moveY>10)
    moveY=10-mypos(2);
end
if (mypos(2)+moveY<0)
    moveY=0-mypos(2);
end
if (mypos(1)+moveX>10)
    moveX=10-mypos(1);
end
if (mypos(1)+moveX<0)
    moveX=0-mypos(1);
end

if(mypos(2)+moveY==10 || mypos(2)+moveY==0)
    moveY=0;
end

if(mypos(1)+moveX==10 || mypos(1)+moveX==0)
    moveX=0;
end

move = [moveX moveY];

end
%%
function tFuel=findMeFuel(mypos,existFuels,opPos) %loc of cloest and relevant fuel
while (true)
    size=length(existFuels);
 if (size==0)
    tFuel = -1;
    return;
 end
mydFuel = distance(mypos,existFuels);
opdFuel = distance(opPos,existFuels);
myclosestFuelIdx = find(min(mydFuel)==mydFuel);
opclosestFuelIdx = find(min(opdFuel)==opdFuel);
if (myclosestFuelIdx==opclosestFuelIdx && mydFuel(myclosestFuelIdx)>opdFuel(opclosestFuelIdx)) % if op is closer than me to the fuel
    if (size>=3)
    tFuel=existFuels(find(max(opdFuel)==opdFuel),:);
    return;
    end
    existFuels(myclosestFuelIdx,:)=[];
else 
    break;
end

end

tFuel= existFuels(myclosestFuelIdx,:);
end
%% 
function [answer,loc] = inKav(existMines,mineR,poly,myPos,myFuel,dx,lmax) %if we have a mine on the way of my fuel
loc=0;
answer = 0;
tmp =myPlot(poly);
index=doesLinearCutCircle(poly,existMines,mineR+0.1);
dangerMines = [];
if (index~=0)
dangerMines = existMines(index,:);

[newPos(1) newPos(2)]= DirectMove(lmax,poly,dx);
newPos(1) = newPos(1) + myPos(1);
newPos(2) = newPos(2) + myPos(2);


dMines = distance(myPos,dangerMines)>distance(newPos,dangerMines);
dMines = dangerMines(find(dMines==1),:);
distMines = distance(myPos,dMines);

if (length(dMines)~=0)
    idx = find(min(distMines)==distMines);
    loc = dMines(idx,:);
    if (distance(myPos,loc) < distance(myPos,myFuel))
    answer =1;
    end
end
end
delete(tmp);

end

%% 
function [moveX moveY] = DirectMove(lmax,m,dx) %calc next move
degree = atan(m(1));
moveX = lmax*cos(degree);
moveY = lmax*sin(degree);
if (dx <0)
    moveX = -moveX;
    moveY = -moveY;
end

end
%%
function [moveX moveY dest dx] = fixMove(lmax,myPos,targetFuel,mSign) %try another way for the my fuel
dx= targetFuel(1) - myPos(1);
dy= targetFuel(2) - myPos(2);
xMiddle= dx/2 +myPos(1);
yMiddle= dy/2 +myPos(2);
deltaX=mSign+xMiddle;
deltaY=mSign+yMiddle;

if(abs(dx)<abs(dy)) %% choice by x
newM= polyfit([deltaX myPos(1)],[yMiddle myPos(2)],1);
dx=deltaX - myPos(1);
dest= [deltaX yMiddle];
else %% choice by x
newM= polyfit([xMiddle myPos(1)],[deltaY myPos(2)],1);
dx=xMiddle - myPos(1);
dest= [xMiddle deltaY];

end

[moveX moveY]=DirectMove(lmax,newM,dx);

end
%%
function tmp = myPlot(m) %for debug
x=linspace(-10,10,100);
y=m(1)*x+m(2);
hold on;
tmp=plot(x,y);
end
%%
function result=doesLinearCutCircle(poly,minePos,mRadius)%find the mine in our way
result=0;
for i=1:length(minePos)
    a=(1+poly(1)^2);
    b=2*poly(1)*(poly(2)-minePos(i,2))-2*minePos(i,1);
    c=(poly(2)-minePos(i,2))^2 +minePos(i,1)^2 -mRadius^2;
    calc(i,:)=roots([a b c])';
    img1=imag(calc(i,1));
    img2=imag(calc(i,2));
    if(img1==0 & img2==0)
        realVec(i)=1;
    else
        realVec(i)=0;
    end
end

index=find(realVec==1);
if (length(index)~=0)
    result=index;
end
end
%%
function result=Attack(myPos,opPos,roboR,env,lmax) %check if we can attack
result=0;
dis=distance(myPos,opPos)-2*roboR;
if((dis<=3*lmax)&&(env.info.fuel> env.info.fuel_op)) 
    result=1;
end
end

%%%%%%%%%
function [moveX moveY]= mainFunc(flag,mFRaidus,lmax, mypos, targetFuel,existMines,moveX,moveY)
k=randi([1 3],1,1)*mFRaidus;
counter=0;
while (flag)
[newMoveX newMoveY targetFuelImg dx]= fixMove(lmax,mypos,targetFuel,k); %changed targetFuel
newPos(1)=mypos(1)+newMoveX;
newPos(2)=mypos(2)+newMoveY;
dx = targetFuelImg(1)-newPos(1);
m= polyfit([targetFuelImg(1) newPos(1)],[targetFuelImg(2) newPos(2)],1);
[flag loc] = inKav(existMines,mFRaidus,m,newPos,targetFuel,dx,lmax);
if(~flag)
    moveX =newMoveX;
    moveY=newMoveY;
    break
end
[newMoveX newMoveY targetFuelImg dx]= fixMove(lmax,mypos,targetFuel,-k);
newPos(1)=mypos(1)+newMoveX;
newPos(2)=mypos(2)+newMoveY;
dx = targetFuelImg(1)-newPos(1);
m= polyfit([targetFuelImg(1) newPos(1)],[targetFuelImg(2) newPos(2)],1);
[flag loc] = inKav(existMines,mFRaidus,m,newPos,targetFuel,dx,lmax);
if(~flag)
    moveX =newMoveX;
    moveY=newMoveY;
    break
end
k=k+mFRaidus;
counter=counter+1;
if (counter==10)
    break;
end
%
end
end