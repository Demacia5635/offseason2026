import 'package:flutter/material.dart';
import 'package:flutter_application_1/paintersState/point_selection_state.dart';
import 'package:flutter_application_1/paintersState/robotPaintState.dart';
import 'package:flutter_application_1/widgets/field_background_image.dart';
import 'package:flutter_application_1/widgets/painter.dart';
import 'package:flutter_application_1/widgets/mousePozesan.dart';
import 'package:flutter_application_1/widgets/displayMousePozesan.dart';

void main() {
  PointSelectionState pointSelectionState= PointSelectionState();
  FieldElementsPainter fieldElementsPainter = FieldElementsPainter(startPoint: , endPoint: ,scale:);
  FieldBackgroundImage fieldBackgroundImage = FieldBackgroundImage(width:1000 ,height: 1000);
  FieldOverlayPainter fieldOverlayPainter = FieldOverlayPainter(width: ,height: );
  InteractiveFieldWidget interactiveFieldWidget =InteractiveFieldWidget();
  MouseCoordinatesDisplay mouseCoordinatesDisplay = MouseCoordinatesDisplay(xMeters: ,yMeters: );

  runApp();

  
}


