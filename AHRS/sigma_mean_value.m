%function：计算Sigma点均值
%chuzhiwei
%2019.09.12
function Xhat = sigma_mean_value(X,W)
   Xhat =  W(1) * X(:,1) + W(2) * X(:,2) + W(3) * X(:,3) + W(4) * X(:,4) + W(5) * X(:,5) + W(6) * X(:,6) + W(7) * X(:,7);
   yaw_max = max(X(1,:));
   yaw_min = min(X(1,:));
   if ((yaw_max - yaw_min) > 180)
       Xhat(1) = 0;
   end
   
%%%%%以下为AHRS在没有航向观测的情况下仅靠陀螺积分，初始阶段（UT变换sigma点较分散）下时使之能趋近初始设定值所做的优化。   
%       Xhat =  W(1) * X(:,1) + W(2) * X(:,2) + W(3) * X(:,3) + W(4) * X(:,4) + W(5) * X(:,5) + W(6) * X(:,6) + W(7) * X(:,7);
%    
%    diff = zeros(6,1);
%    cout = 1;
%    yaw_max = max([X(1,2),X(1,3),X(1,4),X(1,5),X(1,6),X(1,7)]);
%    yaw_min = min([X(1,2),X(1,3),X(1,4),X(1,5),X(1,6),X(1,7)]);
%    if((yaw_max-yaw_min) > 15)           
%        sum_yaw = X(1,2);
%        for i = 3 : 7              
%            diff(i-1) = abs(X(1,2) - X(1,i));
%            if(diff(i-1)<3)
%                cout = cout + 1;
%                sum_yaw = sum_yaw + X(1,i);
%            end
%        end
%        if(cout==1) 
%            sum_yaw = X(1,3);
%            for i = 4 : 7
%                diff(i-1) = abs(X(1,3) - X(1,i));
%                if(diff(i-1)<3)
%                    cout = cout + 1;
%                    sum_yaw = sum_yaw + X(1,i);
%                end
%            end
%        end
%        if(cout==1) 
%            sum_yaw = X(1,4);
%            for i = 5 : 7
%                diff(i-1) = abs(X(1,4) - X(1,i));
%                if(diff(i-1)<3)
%                    cout = cout + 1;
%                    sum_yaw = sum_yaw + X(1,i);
%                end
%            end
%        end
%        if(cout==1) 
%            sum_yaw = X(1,5);
%            for i = 6 : 7
%                diff(i-1) = abs(X(1,5) - X(1,i));
%                if(diff(i-1)<3)
%                    cout = cout + 1;
%                    sum_yaw = sum_yaw + X(1,i);
%                end
%            end
%        end
%        if(cout==1) 
%            sum_yaw = X(1,6);
%            for i = 7
%                diff(i-1) = abs(X(1,6) - X(1,i));
%                if(diff(i-1)<3)
%                    cout = cout + 1;
%                    sum_yaw = sum_yaw + X(1,i);
%                end
%            end
%        end
%        Xhat(1) = sum_yaw/cout;