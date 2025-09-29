import 'package:flutter/material.dart';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text("FRC Field with Background")),
        body: const FRCField(), // Remove Center widget
      ),
    );
  }
}

class FRCField extends StatefulWidget {
  const FRCField({super.key});

  @override
  FRCFieldState createState() => FRCFieldState(); // Removed underscore to make it public
}

class FRCFieldState extends State<FRCField> {
  @override
  Widget build(BuildContext context) {
    return Container(
      // Add your FRC field implementation here
      child: const Text("FRC Field Content"),
    );
  }
}