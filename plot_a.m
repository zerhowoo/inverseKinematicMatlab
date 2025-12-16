 [t1 t2 target]=size(a);
xa=a;
error=zeros(1,target);   
for i=1:target   % i=1:target
 view(-270,0); 
    %view(123,26); 
%  view(90,0);
axis([-200 200 -1300 600 -100 800]);% axis([-400 400 -400 1400 -1000 1200]);
%  axis equal; 
   hold off
plot3(xa(:,1,i), xa(:,2,i),xa(:,3,i),'.-','Markersize',15,'LineWidth',2);
axis equal;
 hold on
  plot3(kp(:,1), kp(:,2),kp(:,3),'.-');
  % view(-270,0); 
view(123,26); 
% view(90,0);
axis([-200 200 -1300 600 -100 800]);
% axis equal; 
 pause(0.01);
 
% for j=1:7
% error(i)=  norm(xa(j,:,i+1)-xa(j,:,i))^2+error(i);
% error_index(i)=error(i)/(norm( kp(i+1,:)-kp(i,:))^2)/7;
%  end
      
end
% plot([1:target-1],error_index);