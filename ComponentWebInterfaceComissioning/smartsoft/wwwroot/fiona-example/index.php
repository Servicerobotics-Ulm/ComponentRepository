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
?>
</head>
<body> 

    <div data-role="page" id="home">
	<?php if(auth())
	{
		printHeader();
	} else {
        	echo '<header data-role="header">';
	        echo '<h1>'.siteName().'</h1>';
	        echo '<a href="skripts/logout.php" data-icon="gear" data-ajax="false" rel="external" class="ui-btn-right">Logout</a>';
	        echo '</header><!-- /header -->';

		}
	?>
        <nav data-role="navbar">
            <ul>
		<?php
                if(auth()){
                        printMenu();
		} else {
		
                	#echo '<li><a href="index.php" data-icon="cus-home" data-theme="c" class="ui-btn-active ui-state-persist" data-transition="fade">Home</a></li>';
		}
		?>
            </ul>
        </nav>
        <div data-role="content" data-theme="d">
            <div class="center">
                <?php if (auth()){?>
		Welcome<p>
                <!--<a href="<?php echo $basepath; ?>skripts/logout.php" data-ajax="false" data-role="button" data-inline="true" data-theme="a">Logout</a>-->
                <?php } else{ ?>
                <img src="<?php echo $basepath; ?>media/servicerobotik_logo-200.png" alt="Service Robotics Ulm" title="Service Robotics Ulm" />
                <p>
		Please log in.<p>
                <a href="#login" data-role="button" data-inline="true" data-theme="b" data-rel="dialog" data-transition="pop">Login</a>
                <?php } ?>
            </div>
        </div>
<?php printFooter(); ?>
    </div><!-- /page -->



    <div data-role="page" id="login">
        <div data-role="header">
            <h1><?=siteName()?> - Login</h1>
        </div><!-- /header -->
        <div data-role="content" data-theme="d">    
	Use master-password "test" to circumvent password check.<br>
            <form action="<?php echo $basepath; ?>skripts/login.php" method="post">
                <fieldset class="ui-grid-a">
                <div data-role="fieldcontain">
                    <label for="host">Host:</label>
                    <input type="text" name="host" id="host" value="http://<?=$_SERVER["SERVER_NAME"]?>"  />
                </div>
                 <div data-role="fieldcontain">
                    <label for="username">Username:</label>
                    <input type="text" name="username" id="username" value="fiona"  />
                </div>  
                <div data-role="fieldcontain">
                    <label for="password">Password:</label>
                    <input type="password" name="password" id="password" value="test" />
                </div>
                
                    <div class="ui-block-a"><a href="#home" data-role="button" data-theme="c">Cancel</a></div>
                    <div class="ui-block-b"><button type="submit" data-ajax="false" data-theme="b">Submit</button></div>       
                </fieldset>
            </form>
        </div><!-- /content -->
    </div><!-- /page -->






</body>
</html>
