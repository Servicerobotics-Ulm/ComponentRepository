//--------------------------------------------------------------------------
//BSD 3-Clause License
//
//  Copyright (C) Servicerobotics Ulm
//  University of Applied Sciences Ulm
//  Prittwitzstr. 10
//  89075 Ulm
//  Germany
//  All rights reserved.
//
//  Author: Nayabrasul Shaik
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------


import QtQuick 2.12
import QtQuick.Window 2.12
//import QtQuick.Layouts 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls 1.4
//import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Styles 1.2
import "."
import "qml_styling"

ApplicationWindow {
    id: root
    visible: true
    width: 640
    height:500
    title: qsTr("Robot Console")     
    property int num_comp: ComponentListModel.size()
    property int component_height : 60
    ListView {
        anchors.fill: parent
        model: ComponentListModel
        snapMode: ListView.NoSnap
        delegate: Rectangle{
            id: rect
            height: component_height
            property string curr_state:model.current_state
            property string curr_comp:model.name
            property string num_states:model.state_list.length==0 ? 1: model.state_list.length
            property bool is_dead: {model.state_list.length ==0 ? true : false}
            property bool has_param_slave : {model.has_param_slave == 0 ? false : true}

            width: parent.width            
            border.color: "white"
            border.width: 2
            Rectangle{
                id: rect_comp_name
                 width: 220
                 height: component_height
                 color: UI_Colors.color_component_bg                 
                 border.width: 1
                 border.color: "white"
                 radius : 10
            Text {
                id: comp_name
                text: model.name
                anchors.verticalCenter:  parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter                
                color: UI_Colors.color_component_name
                font.family: UI_Fonts.component_name_font_family
                font.pixelSize: UI_Fonts.component_name_font_size  
            }
            }
            Column{
                 anchors.left: rect_comp_name.right

            Row{
                id: row_states
               // anchors.left: comp_name.right
                width: rect.width - rect_comp_name.width
                height: rect.has_param_slave ? component_height/2 : component_height
                Repeater{
                    //count: 1
                    id: aa
                    //model:  state_list.length
                    model:  num_states
                    Button {
                        id: id_state_button
                        width: parent.width / num_states;
                        height: parent.height                        
                        text: is_dead? "Not reachable" : state_list[index] 
                        
                        onClicked: { 
                            if(!is_dead)
                            {
                            console.log( "Setting state of " + curr_comp + " to " + state_list[index])
                            ComponentListModel.set_state(curr_comp, state_list[index]);
                            ComponentListModel.refresh();
                            }else{
                            console.log( "Component Not reachable")
                            }
                        }
                        anchors.margins : 5
                        style: ButtonStyle {
                            background: Rectangle {
                                id:bg
                                //implicitWidth: 85
                                //implicitHeight: 40
                                border.width: (state_list[index] == curr_state) ? 2 : 1
                                border.color: (state_list[index] == curr_state) ? UI_Colors.color_state_border_on : UI_Colors.color_state_border_off
                                radius: is_dead? 5: id_state_button.width/5
                                color: is_dead ? UI_Colors.color_state_zombie: state_list[index] == curr_state ? UI_Colors.color_state_on : UI_Colors.color_state_off
                                Component.onCompleted: {
                                //console.log( "bg = " + bg.color)
                                }

                            }
                        }
                        Component.onCompleted:
                        {
//console.log( curr_comp + " is dead = " + text + " index " + index + " num_states" + num_states)
                        }
                    }
                }
            }

            Row{
                anchors.top: row_states.bottom                
                width: rect.width - rect_comp_name.width
                height: rect.has_param_slave ? component_height/2 : 0

                Rectangle{
                    id: param_rect
                    anchors.fill: parent
                    visible: rect.has_param_slave
                    Rectangle{
                    id: rect_input
                    width: (parent.width)*3/4
                    height: parent.height                    
                    border.width: 1
                    border.color: UI_Colors.color_component_bg
                    radius: 5
                TextInput{
                    id: ti_param
                    anchors.fill : parent
                    font.pixelSize: UI_Fonts.font_size_text_input_size
                    font.family: UI_Fonts.font_family_text_input
                    color:UI_Colors.color_text_input_text
                    focus: true
                    text: "Enter ParameterRequest"
                    anchors.topMargin: 5                          
                }
                    }
                Button {
                    //width: ((parent.width)*1/4 <150) ? 150 : (parent.width)*1/4;
                    width: (parent.width)*1/4;
                    anchors.left: rect_input.right
                    height: parent.height                    
                    text: "<font color='#FFFFFF'> Send </font>"
                    onClicked: {
                    console.log("ParameterRequest: " + ti_param.text);
                    ComponentListModel.send_parameter_request(curr_comp, ti_param.text);
                    console.log( "send button width : " + width)                    
                    }                    
                    style: ButtonStyle {
                            background: Rectangle {
                                id:bg                                
                                border.color:  UI_Colors.color_button_rectangle_border
                                radius: 5
                                //color: UI_Colors.color_button_rectangle
                                color: control.down ? UI_Colors.color_button_rectangle_border : UI_Colors.color_button_rectangle
                            }
                        }          
                }
            }
            }

        }

    }
        footer: Rectangle{
            height: component_height/2
            color: UI_Colors.color_button_rectangle
            width: root.width
            border.width: 1
            border.color: UI_Colors.color_button_rectangle_border
            radius: 10
            Text {
                id: name
                text: qsTr("Refresh")                                
                anchors.verticalCenter:  parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: UI_Fonts.refresh_button_font_size
                font.family: UI_Fonts.refresh_button_font_family
                color: UI_Colors.color_button_text
            }
            MouseArea{
                anchors.fill:  parent
                onClicked: {console.log( " click refresh")
                     ComponentListModel.refresh();
                }

            }
        }

    Timer{
        interval: 250; running: false; repeat: true
        onTriggered: {
            ComponentListModel.refresh();
        }
    }

}


}
