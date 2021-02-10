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
include('./skripts/header.php');
$basepath = getBaseUrl(); 
#if (auth()){
?>
<body style='background-color: white'> 
<style>
.ui-page { 
background: white;
}
</style>
    <div data-role="page" id="home">
<header data-role="header" ><h1><?=siteName()?></h1><a href="/fiona-example/skripts/logout.php" data-icon="gear" data-ajax="false" rel="external" class="ui-btn-right">Logout</a></header>
        <div data-role="content" data-theme="d">
            <div class="center">
		<span style="color: red; font-weight: bold">ACCESS DENIED</span>
<p>
                <a href="/fiona-example/#login" data-role="button" data-inline="true" data-theme="b" target="_parent">Login</a>
            </div>
        </div>


        <?php printFooter(); ?>
    </div><!-- /page -->
</body>
</html>
<?php 
#} 
?>
