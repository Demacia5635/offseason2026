import 'package:flutter/material.dart';
//delete

void main() {
  runApp(const FRCFieldApp());
}

class FRCFieldApp extends StatelessWidget {
  const FRCFieldApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'FRC Field Planner',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        useMaterial3: true,
      ),
      //home: const FRCFieldScreen(),
    );
  }
}