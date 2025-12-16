 function newAng=SloveAngles(newLinkCor,LinkNum)
%%%%% 必须有一个根部关节在y轴上，否则计算会出错%%%%%%%
global L;
newLink=zeros(4,4,LinkNum);
newAng=zeros(LinkNum+1,2);
%if abs(newLinkCor(1,1))>0.000000001 ||  abs(newLinkCor(1,3))>0.0000000001  disp 'a1 is not at Y-axis' ;return; end
newAng(LinkNum+1,1)=newLinkCor(1,2)+sum(L);  flag1=1; 
fa1=@(x,z,i)[cos(z) -sin(z) 0 0;sin(z) cos(z) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 cos(x) -sin(x) 0;0 sin(x) cos(x) 0;0 0 0 1]*[1 0 0 0;0 1 0 L(i);0 0 1 0;0 0 0 1];
for i=1:LinkNum

if newLinkCor(i+1,1)==0 && newLinkCor(i+1,3)==0 && flag1==1 
   newAng(i,:)=[0 0]; 
   if i==1 newLink(:,:,1)=fa1(newAng(1,1),newAng(1,2),1);    
   else newLink(:,:,i)=newLink(:,:,i-1)*fa1(newAng(i,1),newAng(i,2),i); 
   end 
continue;end
    
if flag1==1 
       if i==1 
           oCor=newLinkCor(i,2) ;  
       else 
           oCor=newLinkCor(i,2) ; 
       end % oCor=newAng(LinkNum+1,1);   else oCor=newLinkCor(i,2) ; end
       newAng(i,1)= asin(newLinkCor(i+1,3)/sqrt((newLinkCor(i+1,2)-oCor)^2 + newLinkCor(i+1,3)^2));
       newAng(i,2)= -asin(newLinkCor(i+1,1)/sqrt((newLinkCor(i+1,2)-oCor)^2 + newLinkCor(i+1,3)^2));
       flag1=0;
       if i==1 
           newLink(:,:,1)=fa1(newAng(1,1),newAng(1,2),1);    
       else 
           newLink(:,:,i)=newLink(:,:,i-1)*fa1(newAng(i,1),newAng(i,2),i); 
       end
else  

      tPose=[newLink(1:3,3,i-1),newLink(1:3,2,i-1),-newLink(1:3,1,i-1)];
      temptri=tPose\(newLinkCor(i+1,:)'-newLinkCor(i,:)')*(1/L(i));
      newAng(i,1)=asin(temptri(1));
      newAng(i,2)= asin(temptri(3)/cos(newAng(i,1)));   
      newLink(:,:,i)=newLink(:,:,i-1)*fa1(newAng(i,1),newAng(i,2),i); 
end      
end