
var ws_port = 9080;
// ws_ip must be defined in the HTML-part via PHP like:
// var ws_ip = "<?php echo $_SERVER['SERVER_ADDR']; ?>";
var ws_ip = location.hostname; 
var ws_endpoint = "/videostream";
var image_updated = false;

var start_command = "STARTSTREAM";
var stop_command = "STOPSTREAM";


 


window.onload=function(){
  ws=new WebSocket("ws://" + ws_ip + ":" + ws_port + ws_endpoint);
  checkStreaming();

  ws.onmessage=function(evt){
		var image_tag = document.getElementById("video_image");
		image_tag.src = "data:image/jpeg;base64,"+evt.data;
		image_updated = true;

	};
  ws.onopen=function(evt){
	//Start and stop to get one image in the display
    startStopStream(start_command);
  }
}

window.onclose=function(){
  console.log("close conn");
  ws.close();
}

function sendMsgToWs(msg) {
	console.log("send " + msg);
    ws.send(msg);
}

function startStopStream(command = null) {
	if(command == null){
		var start_button = document.getElementById("start_stop_button");	
		command = start_button.value;
	}
	if (command != start_command && command != stop_command){
		return;
	}		
	sendMsgToWs(command);
	//toggleStreamIndication();
}

function toggleStreamIndication() {
	var start_button = document.getElementById("start_stop_button");	
	var button_command = start_button.value;

	var stream_indicator = document.getElementById("stream_indicator");
	
	
	if(button_command == start_command){
		start_button.value = stop_command;
		stream_indicator.style.visibility="visible";
	}else{
		start_button.value = start_command;
		stream_indicator.style.visibility="hidden";
	}
}


////////////
//check stream still running
//TODO use threading z.B. https://t3n.de/news/web-worker-javascript-556677/

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function checkStreaming() {
	
	while(true){
		await sleep(500);
		
		var start_button = document.getElementById("start_stop_button");	

		if(image_updated) {
			image_updated = false;
			if(start_button.value != stop_command){
				console.log('image updated');
			
				start_button.value = stop_command;
				var stream_indicator = document.getElementById("stream_indicator");
				stream_indicator.style.visibility="visible";
			}
		}else{
			if(start_button.value != start_command) {
				console.log('image not updated');
				start_button.value = start_command;
		
				var stream_indicator = document.getElementById("stream_indicator");
				stream_indicator.style.visibility="hidden";
			}
		}
	}

}
