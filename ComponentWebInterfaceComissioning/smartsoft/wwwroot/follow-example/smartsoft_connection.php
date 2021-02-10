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


include('./skripts/functions.php');
#include('./skripts/header.php');
if (auth()){
	// Client
	error_reporting(E_ALL);

	$address = "127.0.0.1";
	$port = 8081;

	//create a tcp/ip socket
	$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
	if($socket === false){
	  echo "socket_create() failed: reason: " . socket_strerror(socket_last_error()) . "\n";
	}else{
	#  echo "socket successfully created.\n";
	}

	#echo "Attempting to connect to '$address' on port '$port'...";
	$result = socket_connect($socket, $address, $port);
	if ($result === false){
	  echo "socket_connect() failed.\nReason: ($result) " . socket_strerror(socket_last_error($socket)) . "\n";
	}else{
	#  echo "successfully connected to $address.\n";
	}

	//$i = 0;
	$i = htmlspecialchars($_GET["task"]);
	#echo "Sending $i to server.\n";
	socket_write($socket, $i, strlen($i));
#echo $i.":".strlen($i);
	$input = socket_read($socket, 2048);
	#echo "Response from server is: $input\n";

	#echo "Closing socket...";
	socket_close($socket);


#header("Cache-Control: no-store, no-cache, must-revalidate, max-age=0");
#header("Cache-Control: post-check=0, pre-check=0", false);
#header("Pragma: no-cache");
//	header('Location: command.php');


	if($_SERVER["HTTP_REFERER"] == "") {
		header('Location: index.php');
	} else {
		header('Location: '.$_SERVER["HTTP_REFERER"]);
	}
/*

<html>
<head>
<meta http-equiv="refresh" content="3; url=command.php" />
</head>
<body>redirecting..</body>
</html>
*/

}?>




