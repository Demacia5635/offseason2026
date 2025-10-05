import 'package:flutter/material.dart';
import 'package:flutter_application_1/back/MouseState.dart';
import 'package:flutter_application_1/back/PointSelectionState.dart';
import 'package:flutter_application_1/back/RobotPaintState.dart';
import 'package:flutter_application_1/front/fieldImage.dart';
import 'package:flutter_application_1/front/mouseCoordinatesDisplay.dart';
import 'package:flutter_application_1/front/painter.dart';

void main() {
    runApp(MaterialApp());
    runApp(MouseCoordinatesDisplay(xMeters: 0, yMeters: 0));
    runApp(Painter(height: 200, width: 200, firstPoint: null, secondPoint: null));
    runApp(FieldImage());
    runApp(MouseState());
    Point_selection_state state = const Point_selection_state();
    state = state.copyWith(firstPoint: const Offset(10, 10));
    RobotPaintState robotPrinter = RobotPaintState(scale: 1.0, startPoint: null, endPoint: null);
    robotPrinter.shouldRepaint(robotPrinter);
}