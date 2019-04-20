function set_monitor_figure(monitor_nr,zeile,spalte)

all_monitors=get( 0,'MonitorPosition');

ausg_monitor=all_monitors(monitor_nr,:);

delta_b=round(ausg_monitor(3)/zeile);
delta_h=round(ausg_monitor(4)/spalte);

%old_pos=get(1,'Position')
       %links, unten, Breite, Hoehe

it=1;
for i_z=1:zeile
links=ausg_monitor(1)+delta_b*(i_z-1);

for i_s=1:spalte

unten=ausg_monitor(2)+delta_h*(i_s-1);
set(it,'Position',[links,unten,delta_b,delta_h]);
it=it+1;
end

end