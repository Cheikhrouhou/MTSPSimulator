function create_newfigure()
global Targets Robots 
global  param
fig_name=sprintf('Multi-Robot Simulator, Result-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv',param);%-%s, cputime
%if(nb_run>1)
    newfig = figure('Name',fig_name,'Numbertitle','off','menubar', 'none');
    p = uipanel('Parent', newfig);    
    %h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.2 0.10 0.75 0.8],'parent',p)
    %drawAll(bt_draw, 'push')
    plot (Targets(:,1),Targets(:,2),'ro')
    hold on
    plot (Robots(1,:),Robots(2,:), 'd')
    
   % put_menu()
    %set (menu_viewnames, 'checked', 'on');
    %view_names(hObject, eventdata, handles)
    %view_names(menu_viewnames, 'checked')
    
%end
end