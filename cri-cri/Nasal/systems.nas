aircraft.livery.init("Aircraft/cri-cri/Models/Liveries", "sim/model/livery/name", "sim/model/livery/index");

var Startup = func{
setprop("controls/electric/battery-switch",1);
setprop("controls/engines/engine[0]/magnetos",3);
setprop("controls/engines/engine[1]/magnetos",3);
setprop("controls/engines/engine[0]/mixture",1);
setprop("controls/engines/engine[1]/mixture",1);
setprop("engines/engine[0]/rpm",2000);
setprop("engines/engine[1]/rpm",2000);
setprop("engines/engine[0]/starter",true);
setprop("engines/engine[1]/starter",true);
setprop("engines/engine[0]/running",1);
setprop("engines/engine[1]/running",1);
}

var Shutdown = func{
setprop("controls/electric/battery-switch",0);
setprop("controls/engines/engine[0]/magnetos",0);
setprop("controls/engines/engine[1]/magnetos",0);
}
