
Requires
========

https://gitlab.com/eidheim/Simple-WebSocket-Server.git

cd Simple-WebSocket-Server
mkdir build
cd build
cmake ..
make
make install


Usage
=====

test.html
---------

<!DOCTYPE HTML>
<html>
  <head>
  </head>
  <body>
	  <image width='480' height='270' id='video_image'></image>
  </body>
  <script src="./video_streaming.js"></script>
</html>


video_streaming.js
------------------

var ws_port = 9080;
// ws_ip must be defined in the HTML-part via PHP like:
// var ws_ip = "<?php echo $_SERVER['SERVER_ADDR']; ?>";
var ws_ip = location.hostname; 
var ws_endpoint = "/videostream";

console.log(ws_ip);

window.onload=function(){
  ws=new WebSocket("ws://" + ws_ip + ":" + ws_port + ws_endpoint);

  ws.onmessage=function(evt){
		var image_tag = document.getElementById("video_image");
		image_tag.src = "data:image/jpeg;base64,"+evt.data;

	};
  ws.onopen=function(evt){
	//Start and stop to get one image in the display
    sendMsgToWs("STARTSTREAM");
    sendMsgToWs("STOPSTREAM");
  }
}

window.onclose=function(){
  console.log("close conn");
  ws.close();
}