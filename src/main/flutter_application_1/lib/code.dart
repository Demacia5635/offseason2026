import 'package:flutter/material.dart';
//delete
class Code extends StatelessWidget {
  const Code({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text("FRC Field with Background")),
        body: const FRCField(),
      ),
    );
  }
}

class FRCField extends StatefulWidget {
  const FRCField({super.key});

  @override
  FRCFieldState createState() => FRCFieldState();
}

class FRCFieldState extends State<FRCField> {
  static const double scale = 20; // 1 meter = 20 pixels

  Offset? start;
  Offset? end;
  bool waitingForSecond = false;

  double mouseX = 0;
  double mouseY = 0;

  void _handleTapDown(TapDownDetails details) {
    setState(() {
      if (!waitingForSecond) {
        start = details.localPosition;
        end = null;
        waitingForSecond = true;
      } else {
        end = details.localPosition;
        waitingForSecond = false;
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    final size = MediaQuery.of(context).size;
    final availableHeight = size.height - kToolbarHeight;

    return Stack(
      children: [
        GestureDetector(
          onTapDown: _handleTapDown,
          child: MouseRegion(
            onHover: (event) {
              setState(() {
                mouseX = event.localPosition.dx / scale;
                mouseY = event.localPosition.dy / scale;
              });
            },
            child: SizedBox(
              width: size.width,
              height: availableHeight,
              child: Stack(
                children: [
                  Image.asset(
                    'assets/frcFiled2025.jpeg',
                    width: size.width,
                    height: availableHeight,
                    fit: BoxFit.cover,
                  ),
                  CustomPaint(
                    size: Size(size.width, availableHeight),
                    painter: FieldPainter(scale, start: start, end: end),
                  ),
                ],
              ),
            ),
          ),
        ),
        Positioned(
          top: 10,
          left: 10,
          child: Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.9),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Text(
              'Mouse X: ${mouseX.toStringAsFixed(2)} m\nMouse Y: ${mouseY.toStringAsFixed(2)} m',
              style: const TextStyle(fontSize: 16),
            ),
          ),
        ),
      ],
    );
  }
}

class FieldPainter extends CustomPainter {
  final double scale;
  final Offset? start;
  final Offset? end;

  FieldPainter(this.scale, {this.start, this.end});

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.red.withOpacity(0.5)
      ..style = PaintingStyle.fill;

    // Example: Red robot 1x1 meter on right side
    canvas.drawRect(
      Rect.fromLTWH(
        size.width - 2 * scale,
        size.height / 2 - 1 * scale,
        1 * scale,
        1 * scale,
      ),
      paint,
    );

    // Draw line if both points exist
    if (start != null && end != null) {
      final linePaint = Paint()
        ..color = Colors.blue
        ..strokeWidth = 3;
      canvas.drawLine(start!, end!, linePaint);
    }
  }

  @override
  bool shouldRepaint(covariant FieldPainter oldDelegate) =>
      oldDelegate.start != start || oldDelegate.end != end;
}

