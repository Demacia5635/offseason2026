import 'package:flutter/material.dart';
import 'package:flutter_application_1/paintersState/point_selection_state.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_application_1/utils/utils.dart';
import 'package:flutter_application_1/widgets/field_background_image.dart';
import 'package:flutter_application_1/widgets/painter.dart';
import 'package:flutter_application_1/widgets/displayMousePozesan.dart';

class InteractiveFieldWidget extends StatefulWidget {
  const InteractiveFieldWidget({super.key});

  @override
  InteractiveFieldWidgetState createState() => InteractiveFieldWidgetState();
}

class InteractiveFieldWidgetState extends State<InteractiveFieldWidget> {
  PointSelectionState _selectionState = const PointSelectionState();
  double _mouseXMeters = 0;
  double _mouseYMeters = 0;

  void _handleFieldTap(TapDownDetails details) {
    setState(() {
      if (!_selectionState.isWaitingForSecondPoint) {
        _selectionState = PointSelectionState(
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
              FieldBackgroundImage(
                width: screenSize.width,
                height: fieldHeight,
              ),
              FieldOverlayPainter(
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
