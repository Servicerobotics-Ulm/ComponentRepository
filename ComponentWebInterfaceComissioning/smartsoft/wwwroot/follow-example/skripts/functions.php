<?php
// --------------------------------------------------------------------------
//
//  Copyright (C) 2015 Dennis Stampfer, Matthias Lutz and others
//
//        Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------

define('TARGET',    "Location: ../index.php" );
define('HOSTSCRIPT', "smartsoft_connection.php");
//define( 'HOST',      "http://c26-09-zafh" );

//hash done http://online-code-generator.com/sha1-hash-with-optional-salt.php
//salt after
//http://192.168.34.106/testing.php/?task=OMFG%20WTF!!!1einself

function siteName() {
	return "HSU Comissioning";
}

function authenticate($user, $pass) {
	//return $user == $pass;
	//return false;


	// Client
	error_reporting(E_ALL);

	$address = "127.0.0.1";
	$port = 8081;

	//create a tcp/ip socket
	$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
	if($socket === false){
	#  echo "socket_create() failed: reason: " . socket_strerror(socket_last_error()) . "\n";
	}else{
	#  echo "socket successfully created.\n";
	}

	#echo "Attempting to connect to '$address' on port '$port'...";
	$result = socket_connect($socket, $address, $port);
	if ($result === false){
	#  echo "socket_connect() failed.\nReason: ($result) " . socket_strerror(socket_last_error($socket)) . "\n";
	}else{
	#  echo "successfully connected to $address.\n";
	}

	//$i = 0;
	$i = htmlspecialchars("AUTH;$user;$pass");
	#echo "Sending $i to server.\n";
	socket_write($socket, $i, strlen($i));
#echo $i.":".strlen($i);
	$input = socket_read($socket, 2048);
	#echo "Response from server is: |$input|\n";
	

	#echo "Closing socket...";
	socket_close($socket);

return trim($input) == "ACK";

}

function login () {
        if(!isset($_SESSION)){
            session_start();
        }
	//$name = explode ( "-",  $_POST['username']);
	//$_SESSION['profile'] = $name[1];
	
	$_SESSION['username'] = $_POST['username'];

        $pass = $_POST['password'];
        // Benutzername und Passwort werden überprüft
        //if ($name[0] == USERNAME && PASSWORD == sha1($pass.SALT)) {

	// CHECKING CREDENTIALS
	// see auth() on how to disable login completely
	if(
		$_POST['password'] == "test" || // for testing
		authenticate($_POST['username'], $_POST['password']) // for SmartSoft Auth service
		//|| true // to accept all
		
	) {
	    $_SESSION['kateIsInTheHouse'] = true;
            $_SESSION['host'] = $_POST['host'];
            header("Expires: Mon, 26 Jul 1997 05:00:00 GMT");
            header("Cache-Control: no-cache");
            header("Pragma: no-cache");

            // Weiterleitung zur geschützten Startseite
            if ($_SERVER['SERVER_PROTOCOL'] == 'HTTP/1.1') {
                if (php_sapi_name() == 'cgi') {
                    header('Status: 200 OK');
                }
                else {
                    header('HTTP/1.1 200 OK');
                }
            }
            header('Location: ../');
            exit;
        } 
	else {
		header('HTTP/1.1 200 OK');
        	header('Location: ../forbidden.php');
        	exit();
	}

}


function logout () {
    if(!isset($_SESSION)){
        session_start();
    }
    session_destroy();
    header('Location: ../');
    exit;
}


function auth(){
//return true; //to ommit login!

    if(!isset($_SESSION)){
        session_start();
    }
    if (!isset($_SESSION['kateIsInTheHouse']) || !$_SESSION['kateIsInTheHouse']) {
        return false;
    }
    return true;
}


function getBaseUrl(){
    // output: /myproject/index.php
    $currentPath = $_SERVER['PHP_SELF'];

    // output: Array ( [dirname] => /myproject [basename] => index.php [extension] => php [filename] => index )
    $pathInfo = pathinfo($currentPath);

    // output: localhost
    $hostName = $_SERVER['HTTP_HOST'];

    // output: http://
    $protocol = strtolower(substr($_SERVER["SERVER_PROTOCOL"],0,5))=='https://'?'https://':'http://';

    // return: http://localhost/myproject/
    return $protocol.$hostName.$pathInfo['dirname']."/";
}

function printMenu(){

		$basepath = getBaseUrl(); 
		$script = basename($_SERVER["SCRIPT_NAME"]);
		$m_command = $script == "command.php" ? 'class="ui-btn-active ui-state-persist"' : "";
		$m_index = $script == "index.php" ? 'class="ui-btn-active ui-state-persist"' : "";
		$m_info = $script == "info.php" ? 'class="ui-btn-active ui-state-persist"' : "";
		$m_navigation = $script == "navigation.php" ? 'class="ui-btn-active ui-state-persist"' : "";
		$m_fiona_demonstrator = $script == "fiona-demonstrator.php" ? 'class="ui-btn-active ui-state-persist"' : "";
		$m_lbs = $script == "follow.php" ? 'class="ui-btn-active ui-state-persist"' : "";

		if($_SESSION['username'] == "krankenschwester"){
                	echo '<li><a href="'.$basepath.'index.php" data-icon="cus-home" data-theme="c" '.$m_index.' data-transition="fade">Home</a></li>';
	                echo '<li><a href="'.$basepath.'command.php" data-icon="cus-command" data-theme="c" '.$m_command.' data-transition="fade">Command</a></li>';
		} else {
                	echo '<li><a target="_parent" href="'.$basepath.'index.php" data-icon="cus-home" data-theme="c" '.$m_index.' data-transition="fade">Home</a></li>';
//                	echo '<li><a target="_parent" href="'.$basepath.'navigation.php" data-icon="cus-walk" data-theme="c" '.$m_navigation.' data-transition="fade">HSU Navigation</a></li>';
                	echo '<li><a target="_parent" href="'.$basepath.'follow.php" data-icon="cus-command" data-theme="c" '.$m_lbs.' data-transition="fade">Command Robot</a></li>';
//			echo '<li><a target="_parent" href="'.$basepath.'fiona-demonstrator.php" data-icon="cus-walk" data-theme="c" '.$m_fiona_demonstrator.' data-transition="fade">FIONA Demonstrator</a></li>';
	                echo '<li><a target="_parent" href="'.$basepath.'info.php" data-icon="cus-info" data-theme="c" '.$m_info.' data-transition="fade">Info</a></li>';

		}
}

function printHeader($title = ""){

	if($_SESSION['username'] == "remote-op")
	{
	?>
	 <style type="text/css">

.ui-bar-a { 
background: linear-gradient(to bottom, #FF0000, #8F2222) #000000;
border: 1px solid #8F2222;
color: #FFFFFF;
font-weight: 700;
text-shadow: 0px -1px 1px #000000;
}
</style>
<?php
	 }
	$basepath = getBaseUrl();
        echo '<header data-role="header" >';
	echo '<h1>'.siteName().'</h1>';
        echo '<a href="'.$basepath.'skripts/logout.php" data-icon="gear" data-ajax="false" rel="external" class="ui-btn-right">Logout</a>';
        echo '</header><!-- /header -->';
}

function printFooter(){
        echo "<div data-role='footer'>
            <h4>".siteName()." ".$title."</h4>
        </div><!-- /footer -->";
}


function dataRetrievalScript($refresh_ms = 250) {
?>

	<script>
	<!-- debugging -->
	function objToString(o){
	    var str='';

	    for(var p in o){
		if(typeof o[p] == 'string'){
		    str+= p + ' : ' + o[p]+'<br>\n';
		}else{
		    str+= p + ': <br>ARRAY';
		}
	    }

	    return str;
	}

	//$('body').append("hello");

	/*
	The following lines will get data from a json object and write
	that data to a global js variable "globalData" which is accessible
	from the whole document.
	It will update all html tags (e.g. span, div) in the following way:
	<span id="VARIABLE">VALUE</span>
	-> e.g. globalData.name=john will bring:<span id="name">john</span>
	*/

	var globalData;
	function getData() {
		$.get("data.json", function(data) {
			globalData = data;
	//		alert(data.age);
	//		$("#lname").text(data.lname);
	//		$("#fname").text(data.fname);
	//		$("#age").text(data.age);

			$("#debug").html(objToString(data));

			for(var p in data){
				$("#" + p).text(data[p]);
			}
		}, "json")
		.fail(function() {$("#debug").html("ERROR RECEIVING DATA");});
	//.fail(function() {alert("Error getting Data");});

	}
	setInterval(getData, <?=$refresh_ms?>);
	//	getData();

	</script>
<?php
}




?>
