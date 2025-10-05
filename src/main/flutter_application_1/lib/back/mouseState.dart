import 'package:flutter/material.dart';
import 'package:flutter_application_1/back/PointSelectionState.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_application_1/utils/utils.dart';
import 'package:flutter_application_1/front/fieldImage.dart';
import 'package:flutter_application_1/front/painter.dart';
import 'package:flutter_application_1/front/mouseCoordinatesDisplay.dart';
//delet
class MouseState extends StatefulWidget {
  const MouseState({super.key});

  @override
  MouseStateState createState() => MouseStateState();
}

class MouseStateState extends State<MouseState> {
  Point_selection_state _selectionState = const Point_selection_state();
  double _mouseXMeters = 0;
  double _mouseYMeters = 0;

  void _handleFieldTap(TapDownDetails details) {
    setState(() {
      if (!_selectionState.isWaitingForSecondPoint) {
        _selectionState = Point_selection_state(
          firstPoint: details.localPosition,
          secondPoint: null,
          isWaitingForSecondPoint: true,
        );
      } else {
        _selectionState = _selectionState.copyWith(
          secondPoint: details.localPosition,
          isWaitingForSecondPoint: false,
        );
      }
    });
  }

  void _updateMousePosition(PointerHoverEvent event) {
    setState(() {
      _mouseXMeters = event.localPosition.dx / Constants.metersToPixelsScale;
      _mouseYMeters = event.localPosition.dy / Constants.metersToPixelsScale;
    });
  }

  @override
  Widget build(BuildContext context) {
    final screenSize = MediaQuery.of(context).size;
    final fieldHeight = _calculateFieldHeight(screenSize);

    return Stack(
      children: [
        _buildInteractiveField(screenSize, fieldHeight),
        _buildMouseCoordinatesDisplay(),
      ],
    );
  }

  double _calculateFieldHeight(Size screenSize) {
    return screenSize.height - kToolbarHeight;
  }

  Widget _buildInteractiveField(Size screenSize, double fieldHeight) {
    return GestureDetector(
      onTapDown: _handleFieldTap,
      child: MouseRegion(
        onHover: _updateMousePosition,
        child: SizedBox(
          width: screenSize.width,
          height: fieldHeight,
          child: Stack(
            children: [
              FieldImage(
                width: screenSize.width,
                height: fieldHeight,
              ),
              Painter(
                width: screenSize.width,
                height: fieldHeight,
                firstPoint: _selectionState.firstPoint,
                secondPoint: _selectionState.secondPoint,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildMouseCoordinatesDisplay() {
    return Positioned(
      top: 10,
      left: 10,
      child: MouseCoordinatesDisplay(
        xMeters: _mouseXMeters,
        yMeters: _mouseYMeters,
      ),
    );
  }
}
