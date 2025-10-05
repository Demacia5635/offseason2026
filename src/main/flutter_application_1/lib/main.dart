import 'package:flutter/material.dart';
import 'package:flutter_application_1/back/MouseState.dart';
import 'package:flutter_application_1/front/fieldImage.dart';
import 'package:flutter_application_1/front/mouseCoordinatesDisplay.dart';
import 'package:flutter_application_1/front/painter.dart';
import 'package:flutter_application_1/utils/utils.dart';

void main() {

  Container(
    child: Row(
      children: [
        MouseState(),
        FieldImage(),
        MouseCoordinatesDisplay(xMeters:10 ,yMeters:5 ),
        Painter(height:Constants.robotHeightPixels ,width:Constants.robotWidthPixels  ,firstPoint:null ,secondPoint: null)
      ],
    )
  );

  runApp(Container());

  /*
    runApp(MaterialApp());
    runApp(MouseCoordinatesDisplay(xMeters: 0, yMeters: 0));
    runApp(Painter(height: 200, width: 200, firstPoint: null, secondPoint: null));
    runApp(FieldImage());
    runApp(MouseState());
    */
}