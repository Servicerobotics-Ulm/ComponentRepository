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

#include('functions.php');
?>
<!DOCTYPE html>
<html>
<head>
<head>
    <meta content="text/html; charset=utf-8" http-equiv="Content-Type" />
    <title><?=siteName()?></title>
    <meta name="viewport" content="width=device-width, minimum-scale=1, maximum-scale=1" /> 
    <meta name="apple-mobile-web-app-capable" content="yes" />
    <meta name="apple-mobile-web-app-status-bar-style" content="black" />
    <meta http-equiv='cache-control' content='no-cache'>
    <meta http-equiv='expires' content='0'>
    <meta http-equiv='pragma' content='no-cache'>
    <link rel="stylesheet"  href="../jquery/jquery.mobile-1.0.1/jquery.mobile-1.0.1.min.css" />
    <script src="../jquery/jquery-1.7.1.min.js"></script>
    <script src="../jquery/jquery.mobile-1.0.1/jquery.mobile-1.0.1.min.js"></script> 
    <style type="text/css">
        /*navigation buttons*/
        .ui-navbar .ui-icon{
            height:27px;
            width:27px;
            margin-left:-14px;
        }
        
        .ui-icon-cus-home {
            background: url('icons/glyphish-icons/icons/53-house.png') center center no-repeat;
        }
        .ui-icon-cus-command {
            background: url('icons/glyphish-icons/icons/116-controller.png') center center no-repeat;
        }
        .ui-icon-cus-navigate {
            background: url('icons/glyphish-icons/icons/73-radar.png') center center no-repeat;
        }
        .ui-icon-cus-walk {
            background: url('icons/glyphish-icons/icons/102-walk.png') center center no-repeat;
        }
        .ui-icon-cus-info {
            background: url('icons/glyphish-icons/icons/64-zap.png') center center no-repeat;
        }
        .ui-icon-cus-remote-op {
            background: url('icons/glyphish-icons/icons/73-radar.png') center center no-repeat;
        }
        /*content area*/
        .ui-content div.center{
            text-align:center;
        }


    </style>
    <script>
    // $('#home').live('pagecreate',function(event){
        // $.mobile.changePage( "../index.php", {
            // type: "post", 
            // reloadPage: true,
        // });
    // });


   </script>
