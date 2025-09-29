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
  _FRCFieldState createState() => _FRCFieldState();
}