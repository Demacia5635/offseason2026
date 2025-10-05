import 'package:flutter/material.dart';
import 'package:flutter_application_1/utils/utils.dart';
import 'package:flutter_application_1/back/RobotPaintState.dart';


class Painter extends StatelessWidget {
  final double width;
  final double height;
  final Offset? firstPoint;
  final Offset? secondPoint;
  

  const Painter({
    super.key,
    required this.width,
    required this.height,
    this.firstPoint,
    this.secondPoint,
  });

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: Size(width, height),
      painter: RobotPaintState(
        scale: (Constants.metersToPixelsScale),
        startPoint: firstPoint,
        endPoint: secondPoint,
      ),
    );
  }
}