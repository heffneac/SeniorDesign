var disconnectTimeout,down=0,tright=0,front=0,right=0;
var oldValues=[0,0,0,0],ctx;
var acceptmoves1=0, acceptmoves2=0;
var watchID;
var geoLoc;
var count=0;
var gps_follow=0;
var latitude,longitude,myLatlng,map,mapSmall;
var isinitMap=0;
var videoip;
var slide=true;
var marker;
var marker1;
var marker1LatLng;
var lat_offset = 0;
var long_offset = 0;
var z1deg,line=[];

$(".battery").click(function(){
    $(".battery-window").toggle(200);
    $(".attitude-window").hide(200);
    $(".gps-window").hide(200);
    $(".connection-window").hide(200);

});
$(".attitude").click(function(){
    $(".attitude-window").toggle(200);
    $(".battery-window").hide(200);
    $(".gps-window").hide(200);
    $(".connection-window").hide(200);

});
$(".gps").click(function(){
    $(".gps-window").toggle(200);
    $(".battery-window").hide(200);
    $(".attitude-window").hide(200);
    $(".connection-window").hide(200);

});
$(".connection-status").click(function(){
    $(".connection-window").toggle(200);
    $(".battery-window").hide(200);
    $(".attitude-window").hide(200);
    $(".gps-window").hide(200);
});
$(document).ready(function(){
    rosInitialize();
    ctx=$("#myCanvas")[0].getContext("2d");
    ctx.beginPath();
    ctx.arc(15,15,13,0,2*Math.PI);
    ctx.lineWidth=3;
    ctx.strokeStyle="#fff";
    ctx.stroke();
    ctx.font="bold 12px Arial";
    ctx.fillStyle="white";
    ctx.fillText("100",5,18);
    function checkWidth(){
        if($(window).width()<$(window).height()){
            var left1= ($(window).width()-200)/2;
            var top1= ((($(window).height()-40)/2)-200)/2+40;
            var left2=left1;
            var top2=3*top1+90;
            $(".joystick-zone1").attr("style","left:" + left1 + "px;top:" + top1 + "px;");
            $(".joystick-zone2").attr("style","left:" + left2 + "px;top:" + top2 + "px;");
            setTimeout(function(){$(".joystick-labels").hide(200);},10000);
            $(".altLabel, .video-box, .select-video-window, .headerLabel").hide();
            $(".joystick-labels").show();
        }
        else if($(window).width() <= 700){
            $(".joystick-zone1").attr("style","left:5px;bottom:0px;");
            $(".joystick-zone2").attr("style","right:5px;bottom:0px;");
            $(".altLabel, #height-data, .select-video-window, .joystick-labels, .headerLabel").hide();
            $(".land-button").attr('style','display:block;right:30%');
            $(".takeoff-button").attr('style','display:block;left:30%');
            $(".select-video-window").attr('style','display:block;left:'+(($(window).width()-350)/2)+'px;');
        }
        else if($(window).width() > 700){
            var left1= (($(window).width()/2)-350)/2;
            var left2=((($(window).width()/2)-250)/2)*3+350;
            var top=(($(window).height()-40)-200)/2+120;
            $(".joystick-zone1").attr("style","left:" + left1 + "px;top:" + top + "px;");
            $(".joystick-zone2").attr("style","left:" + left2 + "px;top:" + top + "px;");
            $(".select-video-window").attr('style','display:block;left:'+(($(window).width()-350)/2)+'px;');
            //setTimeout(function(){$(".joystick-labels").hide(200);},10000);
            $(".altLabel, #height-data, .video-box, .joystick-labels, .headerLabel").show();
            $(".land-button").attr('style','display:block;right:1%');
            $(".takeoff-button").attr('style','display:block;left:10%');
        }
    }
    checkWidth();
    $(window).resize(checkWidth);
})

function socketCallback(){
    var listenerBatteryStatus = new ROSLIB.Topic({
            ros :ros,
            name : '/'+namespace+'/mavros/battery',
            messageType : 'sensor_msgs/BatteryState',
            throttle_rate: 500
    });
    listenerBatteryStatus.subscribe(function(message) {
        $("#voltage").html("Volt: &nbsp &nbsp  &nbsp &nbsp &nbsp"+round(message.voltage,2)+" V");
        $("#current").html("Current:  &nbsp &nbsp &nbsp"+round(message.current,2)+" A");
        $("#remaining").html("Remaining: "+round(message.percentage,2)*100+" %");
        ctx.clearRect(0,0,30,30);
        ctx.beginPath();
        ctx.arc(15,15,13,Math.PI*1.5,(round(message.percentage,2)*2+1.5)*Math.PI);
        ctx.lineWidth=3;
        ctx.strokeStyle="#fff";
        ctx.stroke();
        if ((round(message.percentage,2)*100)>=100)
            ctx.fillText("100",5,18);
        else
            ctx.fillText(round(message.percentage,2)*100,8,18);
    });

    var listenerAttitude = new ROSLIB.Topic({
        ros :ros,
        name : '/'+namespace+'/mavros/imu/data_euler',
        messageType : 'geometry_msgs/TwistStamped',
        throttle_rate: 500
    });
    listenerAttitude.subscribe(function(message){
                // console.log(message);

       $('.roll-data').html("Roll: &nbsp "+ round(message.twist.linear.x,3)+" &#0176;");
       $('.pitch-data').html("Pitch: &nbsp "+ round(message.twist.linear.y,3)+" &#0176;");
       $('.yaw-data').html("Yaw: &nbsp "+ round(message.twist.linear.z,3)+" &#0176;");

        if(round(message.twist.linear.y,3)>-0.75 & round(message.twist.linear.y,3)<0.75 ){
            var calTop=parseInt(50+round(message.twist.linear.y,3)*54);
            $(".land").css({top: calTop+"px"});
        }

        $(".rotate").rotate(round(message.twist.linear.x,3)*(-57.2958));
        $(".compass").rotate(round(message.twist.linear.z,3)*(-57.2958));
    });


    var listenerGlobalPositionRaw = new ROSLIB.Topic({
        ros :ros,
        name : '/'+namespace+'/mavros/global_position/raw/fix',
        messageType : 'sensor_msgs/NavSatFix',
        throttle_rate: 1000
    });

    listenerGlobalPositionRaw.subscribe(function(message) {

        //            if(gpsTimeout)clearTimeout(gpsTimeout);
        $("#hdop").html("HDOP: &nbsp &nbsp &nbsp"+Math.sqrt(message.position_covariance[4]));
        $("#satellites").html("Satellites: &nbsp"+message.status.status);
        $(".sat").html("Sat: "+message.status.status);
    });

    var listenerLocalPosition = new ROSLIB.Topic({
        ros :ros,
        name : '/'+namespace+'/mavros/local_position/local',
        messageType : 'geometry_msgs/TwistStamped',
        throttle_rate: 500
    });

    listenerLocalPosition.subscribe(function(message) {

        $('.alt-data').html("Alt: &nbsp "+ (round(message.twist.linear.z,3)*-1)+" m");
        $('.alt').html("Alt: &nbsp "+ (round(message.twist.linear.z,3)*-1)+" m");
    });


    var listenerState = new ROSLIB.Topic({
        ros :ros,
        name : '/'+namespace+'/flyt/state',
        messageType : 'mavros_msgs/State',
        throttle_rate: 500
    });


    listenerState.subscribe(function(message) {
        $("#connection").html("Connected");
        $(".connection-status").children("img").attr("src","img/connected.png");
        clearTimeout(disconnectTimeout);

        disconnectTimeout=window.setTimeout(function(){
            $("#connection").html("Disconnected");
            $(".connection-status").children("img").attr("src","img/disconnected.png");

        },2500);
    });

}


jQuery.fn.rotate = function(degrees) {
    $(this).css({'-webkit-transform' : 'rotate('+ degrees +'deg)',
                 '-moz-transform' : 'rotate('+ degrees +'deg)',
                 '-ms-transform' : 'rotate('+ degrees +'deg)',
                 'transform' : 'rotate('+ degrees +'deg)'});
    return $(this);
};

function round(value,decimal){
    var x=Math.pow(10,decimal);
    return Math.round(value*x)/x;
}

$(".logout").click(function(){
    localStorage.removeItem("token");
    window.location.replace("index.html");

});

//
// JOYSTICK CONTROLS
//

$(".joystick-zone1").on('touchmove',(function(e){
    var pos=$(this).position();
    for (var i=0;i<e.touches.length;i++){
        if(e.touches[i].pageX>(pos.left-5) && e.touches[i].pageX<(pos.left+205) && e.touches[i].pageY>(pos.top-5) && e.touches[i].pageY<(pos.top+205) ){
            $(".joystick1").attr('style','top:'+(e.touches[i].pageY-pos.top-25)+'px;left:'+(e.touches[i].pageX-pos.left-25)+'px;');

            if (e.touches[i].pageX>(pos.left+125))tright=0.5;
            else if (e.touches[i].pageX<(pos.left+75))tright=-0.5;
            else tright=0;
            if (e.touches[i].pageY>(pos.top+125))down=1;
            else if (e.touches[i].pageY<(pos.top+75))down=-1;
            else down=0;
            $(".nnn").html(tright +" "+down);
        }
    }

}));

$(".joystick-zone1").on('touchend',(function(e){

    $(".joystick1").attr('style','top:75px;left:75px;');
    tright=0;
    down=0;
}));

$(".joystick-zone1").mousedown(function(){
    acceptmoves1=1;
});

$(".joystick-zone1").mousemove(function(e){

    var pos=$(this).position();
    if(acceptmoves1){console.log(e.pageX);
        if(e.pageX>(pos.left-5) && e.pageX<(pos.left+205) && e.pageY>(pos.top-5) && e.pageY<(pos.top+205) ){
            $(".joystick1").attr('style','top:'+(e.pageY-pos.top-25)+'px;left:'+(e.pageX-pos.left-25)+'px;');

            if (e.pageX>(pos.left+125))tright=0.5;
            else if (e.pageX<(pos.left+75))tright=-0.5;
            else tright=0;
            if (e.pageY>(pos.top+125))down=1;
            else if (e.pageY<(pos.top+75))down=-1;
            else down=0;
            $(".nnn").html(tright +" "+down);
        }else{
            acceptmoves1=0;
            $(".joystick1").attr('style','top:75px;left:75px;');
            tright=0;
            down=0;
        }
    }

});

$('body').mouseup(function(){

    acceptmoves1=0;
    $(".joystick1").attr('style','top:75px;left:75px;');
    tright=0;
    down=0;
    acceptmoves2=0;
    $(".joystick2").attr('style','top:75px;left:75px;');
    right=0;
    front=0;

});

$(".joystick-zone2").mousedown(function(){
    acceptmoves2=1;
});

$(".joystick-zone2").mousemove(function(e){

    var pos=$(this).position();
    if(acceptmoves2){console.log(e.pageX);
        if(e.pageX>(pos.left-5) && e.pageX<(pos.left+205) && e.pageY>(pos.top-5) && e.pageY<(pos.top+205) ){
            $(".joystick2").attr('style','top:'+(e.pageY-pos.top-25)+'px;left:'+(e.pageX-pos.left-25)+'px;');

            if (e.pageX>(pos.left+125))right=1;
            else if (e.pageX<(pos.left+75))right=-1;
            else right=0;
            if (e.pageY>(pos.top+125))front=-1;
            else if (e.pageY<(pos.top+75))front=1;
            else front=0;
            $(".nnn").html(tright +" "+down);
        }else{
            acceptmoves2=0;
            $(".joystick2").attr('style','top:75px;left:75px;');
            tright=0;
            down=0;
        }
    }

});
    
$(".joystick-zone2").on('touchmove',(function(e){

    var pos=$(this).position();
//    console.log(e.touches);
    for (var i=0;i<e.touches.length;i++){
        if(e.touches[i].pageX>(pos.left-5) && e.touches[i].pageX<(pos.left+205) && e.touches[i].pageY>(pos.top-5) && e.touches[i].pageY<(pos.top+205) ){
            $(".joystick2").attr('style','top:'+(e.touches[i].pageY-pos.top-25)+'px;left:'+(e.touches[i].pageX-pos.left-25)+'px;');

            if (e.touches[i].pageX>(pos.left+125))right=1;
            else if (e.touches[i].pageX<(pos.left+75))right=-1;
            else right=0;
            if (e.touches[i].pageY>(pos.top+125))front=-1;
            else if (e.touches[i].pageY<(pos.top+75))front=1;
            else front=0;
            $(".nnn").html(right +" "+front);
        }
    }

}));
    
$(".joystick-zone2").on('touchend',(function(e){
    $(".joystick2").attr('style','top:75px;left:75px;');
    right=0;
    front=0;
}));


setInterval(callVelocity,1500);

function callVelocity(){
    var newValues=[front,right,down,tright];
    if (newValues.toString()!=oldValues.toString()){
        if (newValues.toString()==="0,0,0,0"){
            positionHold();
            velocitySetpoint(newValues);
        }
        else
            velocitySetpoint(newValues);
        oldValues=newValues;
    }
}

function velocitySetpoint(values){
    var msgdata={};
    msgdata["twist"]={};
    msgdata.twist["twist"]={};
    msgdata.twist.twist["linear"]={};
    msgdata.twist.twist.linear["x"]=parseFloat(values[0]);
    msgdata.twist.twist.linear["y"]=parseFloat(values[1]);
    msgdata.twist.twist.linear["z"]=parseFloat(values[2]);
    msgdata.twist.twist["angular"]={};
    msgdata.twist.twist.angular["z"]=parseFloat(values[3]);
    msgdata["tolerance"]=2.00;
    msgdata["async"]=true;
    msgdata["relative"]=false;
    msgdata["body_frame"]=true;
    if (values[3]==0 & (values[0]!=0 | values[1]!=0 | values[2]!=0))msgdata["yaw_rate_valid"]=false;
    else msgdata["yaw_rate_valid"]=true;

    $.ajax({
        type: "POST",
        dataType: "json",
        headers: { 'Authentication-Token': token },
        data: JSON.stringify(msgdata),
        url: restPath+"/ros/"+namespace+"/navigation/velocity_set",
        success: function(data){
            console.log(data);
            if (data.success){
                console.log("velocity set -- "+ msgdata.twist.twist.linear.x+" "+ msgdata.twist.twist.linear.y+" "+ msgdata.twist.twist.linear.z);
            }
        }
    });

}

function positionHold(){
    $.ajax({
        type: "GET",
        headers: { 'Authentication-Token': token },
        dataType: "json",
        url: restPath+"/ros/"+namespace+"/navigation/position_hold",
        success: function(data){
            if (data.success){
                console.log("position hold");
            }
        }
    });

}

$(".takeoff-button").click(function(){
    var msgdata={};
    msgdata["takeoff_alt"]=parseFloat($("#height-data").val());
    $.ajax({
       type: "POST",
       headers: { 'Authentication-Token': token },
       dataType: "json",
       data: JSON.stringify(msgdata),
       url: restPath+"/ros/"+namespace+"/navigation/take_off",
       success: function(data){console.log(data);
           if(data.success){
                $(".toast").html("System Taking off!!");
                $(".toast").show();
                setTimeout(function(){
                    $(".toast").hide(20);
                },3000);

           }
           else{
                $(".toast").html("Take Off Rejected! Retry!!");
                $(".toast").show();
                setTimeout(function(){
                    $(".toast").hide(20);
                },3000);

           }
       },
       error: function(){
            $(".toast").html("Failed to contact FlytPOD! Retry!!");
            $(".toast").show();
            setTimeout(function(){
                $(".toast").hide(20);
            },3000);
       }
   });
    $(".takeoff-land-window").hide(200);
    $(".height-window").toggle(200);
    $(".container").animate({
        height:'100px'
    },200);

})
$(".land-button").click(function(){
    $.ajax({
           type: "GET",
           headers: { 'Authentication-Token':token },
           dataType: "json",
           url: restPath+"/ros/"+namespace+"/navigation/land",
           success: function(data){

               if(data.success){
                    $(".toast").html("System Landing!");
                    $(".toast").show();
                    setTimeout(function(){
                        $(".toast").hide(20);
                    },3000);
               }
               else{
                    $(".toast").html("Land Rejected by FlytPOD! Retry!!");
                    $(".toast").show();
                    setTimeout(function(){
                        $(".toast").hide(20);
                    },3000);
               }
           },
           error: function(){
                $(".toast").html("Failed to contact FlytPOD! Retry!!");
                $(".toast").show();
                setTimeout(function(){
                    $(".toast").hide(20);
                },3000);
           }
    });

       $(".takeoff-land-window").hide(200);
    $(".height-window").toggle(200);
    $(".container").animate({
        height:'100px'
    },200);
})

$(".nudge").click(function(){

    $(".nudge-box").toggle(100);
});
$(".video").click(function(){
    $(".video-box").toggle();
    $(".select-video-window").toggle();
});

//
// VIDEO STREAMING CONTROLS
//

function get_video_list(){
  var msgdata={};
  msgdata["namespace"]=namespace;

  $.ajax({
   type: "POST",
   headers: { 'Authentication-Token': token },
   dataType: "json",
   contentType: "application/json",
   data: JSON.stringify(msgdata),
   url: restPath+"/list_streams",
   success: function(data){
        for(var i=1;i<(Object.keys(data).length+1);i++){
            $("#video-select").append($('<option>', {
                value: i,
                text: data["stream"+i]
            }));
        }
     },
     error: function(){
         console.log("Error in list_streams");
     }
  });
}

$("#video-select").change(function(){
    videoLink=$(this).children(":selected").html()+"";
});

$(".start-video").click(function(){
    $(".select-video-window").show();
    if(videoLink==""|videoLink=="none"){
        $(".video-box").html("");
    }else{
        $(".video-box").html("<img id='video-page-img' src='' width=100% >");
        $("#video-page-img").attr("src",restPath+":8080/stream?topic="+videoLink+"&width=320&height=240&type=ros_compressed");
        $(".video-page-div").show(200);
    }
});

$(".close-video").click(function(){
    // $(".video-box").toggle();
    // $(".select-video-window").toggle();
    $(".video-box").html("");

})