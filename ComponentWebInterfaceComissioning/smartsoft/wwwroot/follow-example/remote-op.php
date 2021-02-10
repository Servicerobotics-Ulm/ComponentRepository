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
if (auth()){?>
<body> 
    <div data-role="page" id="info">
	<?php 
	printHeader();
	?>
        <nav data-role="navbar">
            <ul>
		<?php 
                printMenu();
		?>
            </ul>
        </nav>
        <div data-role="content" data-theme="d">    
            <div class="center">
                <p>Remote OP</p>
            </div>
            <form action="form.php" method="post">
                <div data-role="controlgroup">
			<?php 
			   echo  '<a href="'.HOSTSCRIPT.'?task=(InterceptTask joystickNavigation)" data-ajax="true" data-role="button">Intercept execution and move with joystick</a>';		
			   echo  '<a href="'.HOSTSCRIPT.'?task=(ResumeTask)" data-ajax="true" data-role="button">Continue Task</a>';		
			?>
                </div>
            </form> 
        </div><!-- /content -->
        <div data-role="footer">
            <h4>Page Footer</h4>
        </div><!-- /footer -->
    </div><!-- /page -->
</body>
</html>
<?php } ?>
