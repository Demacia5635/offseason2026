import 'package:flutter/material.dart';

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
  FRCFieldState createState() => FRCFieldState(); // Removed underscore
}

class FRCFieldState extends State<FRCField> { // Removed underscore
  static const double scale = 20; // 1 meter = 20 pixels
  static const double fieldLength = 27.0;
  static const double fieldWidth = 16.54;

  double mouseX = 0;
  double mouseY = 0;

  @override
  Widget build(BuildContext context) {
    // Get the full available size
    final size = MediaQuery.of(context).size;
    final availableHeight = size.height - kToolbarHeight; // Subtract AppBar height
    
    return Stack(
      children: [
        // Mouse tracking - now fills entire available space
        MouseRegion(
          onHover: (event) {
            setState(() {
              mouseX = event.localPosition.dx / scale;
              mouseY = event.localPosition.dy / scale;
            });
            // Print the position to console
            print('Mouse X: ${mouseX.toStringAsFixed(2)} m, Mouse Y: ${mouseY.toStringAsFixed(2)} m');
          },
          child: SizedBox(
            width: size.width,        // Full screen width
            height: availableHeight,  // Full available height
            child: Stack(
              children: [
                // Background image - fills entire available space
                Image.asset(
                  'assets/frcFiled2025.jpeg',
                  width: size.width,
                  height: availableHeight,
                  fit: BoxFit.cover, // This will cover the entire space
                ),
                // Drawing robots or markers
                CustomPaint(
                  size: Size(size.width, availableHeight),
                  painter: FieldPainter(scale),
                ),
              ],
            ),
          ),
        ),
        // Display mouse position on screen
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

  FieldPainter(this.scale);

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

    // Add more robots or markers if needed
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}