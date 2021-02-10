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
    <div data-role="page" id="command">
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
                <div data-role="controlgroup">
			<?php 
			include 'config.php';
			foreach($feld as $key => $wert){ 
			   echo  '<a href="'.HOSTSCRIPT.'?task='.$key.'" data-ajax="true" data-role="button" target="_parent">'.$wert.'</a>';		
			}
			
			?>
                </div>
	    <br>
	    <center> <h3>Call robot to room:</h3> </center>
	    <form action="<?php echo $basepath.HOSTSCRIPT; ?>" method="get">
		<div data-role="fieldcontain">
			<label for="task" class="select"></label>
			<select name="task" id="task">
			<?php
                        include 'config.php';
			foreach($roomlist as $key => $wert){
                           echo  '<option value="'.$key.'">'.$wert.'</option>';
                        }
			?>
			</select>
		</div>
		<button type="submit" data-ajax="false" data-theme="b">GOTO ROOM</button>
            </form> 
        </div><!-- /content -->
        <div data-role="footer">
            <h4>Page Footer</h4>
        </div><!-- /footer -->
    </div><!-- /page -->
</body>
</html>
<?php } ?>
